"""Target-loss relocalization support helpers.

The high-level loss handler remains in runtime.py because it coordinates the
active queue and completion state. This module contains the pure support and
identity-validation pieces used by that handler.
"""

from __future__ import annotations

import math
from typing import Any

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.common.detection_policy import (
    is_large_structure_stage,
)
from agent.functions.fast_slow.completion_pipeline import (
    CompletionPipeline,
    DetectionDepthBundle,
)
from agent.functions.memory import is_view_relative_stage
from agent.functions.memory.geometry import (
    has_surface_geometry,
    nearest_instance_surface_point,
)
from agent.functions.memory.spatial_reasoning import relation_kind
from agent.functions.perception import detection_is_excluded_by_bearing


def _debug_logs_enabled() -> bool:
    fast_slow_cfg = {
        **(cfg.get("FAST_SLOW", {}) or {}),
        **function_section(cfg, "FAST_SLOW"),
    }
    return bool(fast_slow_cfg.get("DEBUG_LOGS", False))


def _debug_print(message: str) -> None:
    if _debug_logs_enabled():
        print(message)


def _is_above_stage(stage: Any) -> bool:
    return relation_kind(stage) == "above"


def _large_target_expected_offscreen(objects, stage, current_world) -> tuple[bool, str]:
    """Recognize normal facade disappearance without attempting a yaw scan."""
    memory = getattr(objects, "mission_memory", None)
    instance = memory.primary_instance(stage) if memory is not None else None
    if instance is None:
        return False, "large target has no metric lock yet"
    if _is_above_stage(stage):
        overhead = memory.above_overhead_context(stage, current_world)
        if bool((overhead or {}).get("active", False)):
            return True, f"above_{(overhead or {}).get('phase', 'overhead')}"
        roof = memory.roof_navigation_context(stage, current_world)
        if roof is not None:
            return True, "roof acquisition uses down view"

    estimate = memory.estimate_distance(stage, current_world)
    distance_m = float((estimate or {}).get("distance_m", float("inf")))
    close_threshold = max(
        4.0,
        float((getattr(memory, "config", {}) or {}).get("LARGE_EXPECTED_OFFSCREEN_NEAR_M", 14.0)),
    )
    if distance_m <= close_threshold:
        return True, f"locked facade is close ({distance_m:.1f}m)"

    target_world = (
        nearest_instance_surface_point(current_world, instance)
        if has_surface_geometry(instance)
        else list(instance.target_world)
    )
    horizontal = math.hypot(
        float(target_world[0]) - float(current_world[0]),
        float(target_world[1]) - float(current_world[1]),
    )
    down_delta = float(target_world[2]) - float(current_world[2])
    elevation_down_deg = math.degrees(math.atan2(down_delta, max(horizontal, 0.1)))
    offscreen_deg = float(
        (getattr(memory, "config", {}) or {}).get("LARGE_EXPECTED_OFFSCREEN_DOWN_ANGLE_DEG", 32.0)
    )
    if down_delta > 0.0 and elevation_down_deg >= offscreen_deg:
        return True, f"target is {elevation_down_deg:.1f}deg below front view"
    return False, "facade should still be front-visible"


def _relocalization_memory_is_reliable(objects, stage, current_world) -> bool:
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not memory.has_primary(stage):
        return False
    locked_fn = getattr(memory, "is_primary_locked", None)
    if not callable(locked_fn) or not locked_fn(stage):
        return False
    estimate = memory.estimate_distance(stage, current_world)
    if estimate is None:
        return False
    memory_cfg = getattr(memory, "config", {}) or {}
    return bool(
        float(estimate.get("confidence", 0.0) or 0.0)
        >= float(memory_cfg.get("RELOCALIZATION_MEMORY_MIN_CONFIDENCE", 0.35))
        and float(estimate.get("uncertainty_m", 999.0) or 999.0)
        <= float(memory_cfg.get("RELOCALIZATION_MEMORY_MAX_UNCERTAINTY_M", 8.0))
        and float(estimate.get("observation_age_s", 999.0) or 999.0)
        <= float(memory_cfg.get("RELOCALIZATION_MEMORY_MAX_AGE_S", 60.0))
    )


def _build_locked_relocalization_validator(objects, client, stage):
    """Accept only the locked entity, not merely another matching noun."""
    memory = getattr(objects, "mission_memory", None)

    def validate(_stage, front_image, down_image, _front_det, _down_det, candidate):
        if candidate is None or not bool(getattr(candidate, "visible", False)):
            return None
        observer_world, observer_yaw = client.get_pose()
        view = str(getattr(candidate, "camera", "front") or "front").lower()
        candidate_camera_id = str(getattr(candidate, "camera_id", "") or "")
        image = front_image
        for candidate_image in (front_image, down_image):
            camera_frame = getattr(candidate_image, "camera_frame", None) if candidate_image is not None else None
            if camera_frame is not None and candidate_camera_id == str(camera_frame.camera_id):
                image = candidate_image
                break
        else:
            if view.startswith("down"):
                image = down_image
        if memory is not None:
            exclusions = memory.previous_entity_exclusions(stage, observer_world, observer_yaw)
            intrinsics = getattr(getattr(candidate, "camera_frame", None), "rgb_intrinsics", None)
            fov = (
                float(intrinsics.horizontal_fov_deg)
                if intrinsics is not None
                else float((getattr(memory, "sim_config", {}) or {}).get("FRONT_FOV", 90.0))
            )
            if detection_is_excluded_by_bearing(
                candidate,
                image,
                exclusions,
                horizontal_fov_deg=fov,
                observer_yaw_deg=observer_yaw,
            ):
                _debug_print("  [RelocalizeIdentity] rejected=previous_entity_bearing")
                return None
        if memory is not None and hasattr(memory, "evaluate_relocalization_identity"):
            identity = memory.evaluate_relocalization_identity(
                stage,
                candidate,
                image,
                observer_world=observer_world,
                observer_yaw_deg=observer_yaw,
                view=view,
            )
            if not bool(identity.get("accepted", False)):
                _debug_print(
                    "  [RelocalizeIdentity] "
                    f"candidate={getattr(candidate, 'label', '')!r} "
                    f"score={float(getattr(candidate, 'score', 0.0) or 0.0):.2f} "
                    f"rejected={identity.get('reason')} details={identity}"
                )
                return None
        return candidate

    return validate


def _bundle_from_relocalization_result(result) -> DetectionDepthBundle:
    detection = getattr(result, "detection", None)
    view = str(getattr(detection, "camera", "none") or "none").lower()
    # Only the validator-approved physical instance may enter MissionMemory.
    # Keeping the other raw same-class boxes here would let the normal update
    # selector choose a higher-DINO-score candidate that relocalization had
    # explicitly rejected.
    front_image = getattr(result, "front_image", None)
    down_image = getattr(result, "down_image", None)
    detection_camera_id = str(getattr(detection, "camera_id", "") or "")
    front_camera_id = str(getattr(getattr(front_image, "camera_frame", None), "camera_id", "") or "")
    down_camera_id = str(getattr(getattr(down_image, "camera_frame", None), "camera_id", "") or "")
    belongs_down = bool(
        detection is not None
        and (
            detection_camera_id and detection_camera_id == down_camera_id
            or not detection_camera_id and view == "down"
        )
    )
    front_detection = detection if detection is not None and not belongs_down else None
    down_detection = detection if belongs_down else None
    front_detections = [detection] if front_detection is not None else []
    down_detections = [detection] if down_detection is not None else []
    observer_world = getattr(result, "observer_world", None)
    return DetectionDepthBundle(
        best_detection=detection,
        front_detection=front_detection,
        down_detection=down_detection,
        front_detections=front_detections,
        down_detections=down_detections,
        front_image=front_image,
        down_image=down_image,
        front_depth=getattr(result, "front_depth", None),
        down_depth=getattr(result, "down_depth", None),
        observer_world=(list(observer_world) if observer_world is not None else None),
        observer_yaw_deg=getattr(result, "observer_yaw_deg", None),
        capture_timestamp_s=float(getattr(result, "capture_timestamp_s", 0.0) or 0.0),
        camera_frames={
            str(frame.camera_id): frame
            for frame in (
                getattr(front_image, "camera_frame", None),
                getattr(down_image, "camera_frame", None),
            )
            if frame is not None
        },
        distance_view=(detection_camera_id or view) if detection is not None else "none",
        distance_reason="validated_relocalization",
    )
