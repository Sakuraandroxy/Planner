"""Target identity, prebinding, and bearing state for the fast-slow runtime.

The functions in this module are intentionally side-effect compatible with the
original runtime helpers. They own target identity bookkeeping only; planning
and flight execution remain in runtime.py.
"""

from __future__ import annotations

from typing import Any
from types import SimpleNamespace

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.common.detection_policy import (
    detection_reliability as shared_detection_reliability,
)
from agent.functions.fast_slow.completion_pipeline import (
    CompletionPipeline,
    DetectionDepthBundle,
)
from agent.functions.memory import is_view_relative_stage
from agent.functions.perception import detection_is_excluded_by_bearing
from agent.functions.relocalization import TargetLostRecoveryState


def _runtime_stage_key(stage) -> tuple:
    return CompletionPipeline.stage_key(stage)


def _detection_reliability(stage: Any, detection: Any, image: Any = None) -> float:
    return shared_detection_reliability(stage, detection, image)


def _debug_logs_enabled() -> bool:
    fast_slow_cfg = {
        **(cfg.get("FAST_SLOW", {}) or {}),
        **function_section(cfg, "FAST_SLOW"),
    }
    return bool(fast_slow_cfg.get("DEBUG_LOGS", False))


def _debug_print(message: str) -> None:
    if _debug_logs_enabled():
        print(message)



def _target_lost_recovery(objects) -> TargetLostRecoveryState:
    recovery = getattr(objects, "target_lost_recovery", None)
    if recovery is None:
        memory_cfg = dict(getattr(getattr(objects, "mission_memory", None), "config", {}) or {})
        relocalization_cfg = {
            **memory_cfg,
            **dict(function_section(cfg, "RELOCALIZATION") or {}),
        }
        recovery = TargetLostRecoveryState(relocalization_cfg)
        setattr(objects, "target_lost_recovery", recovery)
    return recovery


def _generation_map(objects, name: str) -> dict:
    values = getattr(objects, name, None)
    if values is None:
        values = {}
        setattr(objects, name, values)
    return values


def _stage_generation(objects, stage) -> int:
    return int(_generation_map(objects, "stage_generations").get(_runtime_stage_key(stage), 0))


def _lock_generation(objects, stage) -> int:
    return int(_generation_map(objects, "lock_generations").get(_runtime_stage_key(stage), 0))


def _bump_stage_generation(objects, stage) -> int:
    key = _runtime_stage_key(stage)
    generations = _generation_map(objects, "stage_generations")
    generations[key] = int(generations.get(key, 0)) + 1
    return generations[key]


def _bump_lock_generation(objects, stage) -> int:
    key = _runtime_stage_key(stage)
    generations = _generation_map(objects, "lock_generations")
    generations[key] = int(generations.get(key, 0)) + 1
    return generations[key]


def _locked_instance_id(objects, stage) -> str:
    memory = getattr(objects, "mission_memory", None)
    if memory is None:
        return ""
    instance = memory.primary_instance(stage)
    locked_fn = getattr(memory, "is_primary_locked", None)
    if instance is None or not callable(locked_fn) or not locked_fn(stage):
        return ""
    return str(getattr(instance, "instance_id", "") or "")


def _active_relocalization_session_id(objects, stage) -> str:
    recovery = _target_lost_recovery(objects)
    session = recovery.sessions.get(_runtime_stage_key(stage))
    return str(getattr(session, "session_id", "") or "") if session is not None else ""


def _target_lost_event_is_stale(objects, stage, event) -> tuple[bool, str]:
    """Reject a TargetLost result captured before newer navigation identity state."""
    if event is None:
        return False, ""
    event_stage_generation = int(getattr(event, "stage_generation", 0) or 0)
    current_stage_generation = _stage_generation(objects, stage)
    if event_stage_generation != current_stage_generation:
        return True, f"stage_generation {event_stage_generation}!={current_stage_generation}"
    event_lock_generation = int(getattr(event, "lock_generation", 0) or 0)
    current_lock_generation = _lock_generation(objects, stage)
    if event_lock_generation != current_lock_generation:
        return True, f"lock_generation {event_lock_generation}!={current_lock_generation}"
    event_instance = str(getattr(event, "locked_instance_id", "") or "")
    current_instance = _locked_instance_id(objects, stage)
    if event_instance != current_instance:
        return True, f"locked_instance {event_instance or 'none'}!={current_instance or 'none'}"
    event_session = str(getattr(event, "session_id", "") or "")
    current_session = _active_relocalization_session_id(objects, stage)
    if event_session and event_session != current_session:
        return True, f"session {event_session}!={current_session or 'none'}"

    capture_timestamp = float(
        getattr(getattr(event, "bundle", None), "capture_timestamp_s", 0.0) or 0.0
    )
    if capture_timestamp > 0.0:
        tracker = getattr(objects, "target_bearing_tracker", None)
        latest_bearing = (
            float(tracker.latest_observed_at(_runtime_stage_key(stage)))
            if tracker is not None and hasattr(tracker, "latest_observed_at")
            else 0.0
        )
        memory = getattr(objects, "mission_memory", None)
        primary = memory.primary_instance(stage) if memory is not None else None
        latest_memory = float(getattr(primary, "last_seen_s", 0.0) or 0.0)
        latest_valid = max(latest_bearing, latest_memory)
        if latest_valid > capture_timestamp:
            return True, f"newer_target_observation {latest_valid:.3f}>{capture_timestamp:.3f}"
    return False, ""


def _identity_approved_detections(objects, stage, bundle, view: str) -> list:
    if bundle is None:
        return []
    view_name = str(view or "front").lower()
    detections = list(getattr(bundle, f"{view_name}_detections", None) or [])
    fallback = getattr(bundle, f"{view_name}_detection", None)
    if not detections and fallback is not None:
        detections = [fallback]
    detections = [d for d in detections if d is not None and getattr(d, "visible", False)]
    memory = getattr(objects, "mission_memory", None)
    exclusions = (
        memory.previous_entity_exclusions(stage, bundle.observer_world, bundle.observer_yaw_deg)
        if memory is not None
        and bundle.observer_world is not None
        and bundle.observer_yaw_deg is not None
        else []
    )
    image = getattr(bundle, f"{view_name}_image", None)
    approved = []
    rejected_reasons = []
    for detection in detections:
        intrinsics = getattr(getattr(detection, "camera_frame", None), "rgb_intrinsics", None)
        fov = (
            float(intrinsics.horizontal_fov_deg)
            if intrinsics is not None
            else float((getattr(memory, "sim_config", {}) or {}).get("FRONT_FOV", 90.0))
        )
        if detection_is_excluded_by_bearing(
            detection,
            image,
            exclusions,
            horizontal_fov_deg=fov,
            observer_yaw_deg=bundle.observer_yaw_deg,
        ):
            rejected_reasons.append("previous_entity_bearing")
            continue
        if (
            memory is not None
            and bundle.observer_world is not None
            and bundle.observer_yaw_deg is not None
        ):
            identity = memory.evaluate_locked_detection_identity(
                stage,
                detection,
                image,
                observer_world=bundle.observer_world,
                observer_yaw_deg=bundle.observer_yaw_deg,
                view=view_name,
            )
            if not bool(identity.get("accepted", False)):
                rejected_reasons.append(str(identity.get("reason", "locked_identity_mismatch")))
                _debug_print(
                    "  [TargetIdentity] "
                    f"view={view_name} rejected={identity.get('reason')} "
                    f"details={identity}"
                )
                continue
        approved.append(detection)
    if not approved:
        if detections and rejected_reasons:
            _debug_print(
                f"  [TargetIdentity] all {view_name} detections rejected "
                f"reasons={','.join(sorted(set(rejected_reasons)))}"
            )
        return []
    return approved


def _select_identity_approved_front_detection(objects, stage, bundle):
    approved = _identity_approved_detections(objects, stage, bundle, "front")
    if not approved:
        return None
    return max(approved, key=lambda d: _detection_reliability(stage, d, bundle.front_image))


def _select_identity_approved_down_detection(objects, stage, bundle):
    approved = _identity_approved_detections(objects, stage, bundle, "down")
    if not approved:
        return None
    return max(approved, key=lambda d: _detection_reliability(stage, d, bundle.down_image))


def _select_prebind_front_detection(objects, stage, bundle):
    """Select the first RGB candidate without creating a metric landmark."""

    approved = _identity_approved_detections(objects, stage, bundle, "front")
    approved = [
        detection
        for detection in approved
        if _detection_reliability(stage, detection, bundle.front_image) > 0.0
        and (
            not is_view_relative_stage(stage)
            or _detection_matches_view_relative_sector(
                stage,
                detection,
                bundle.front_image,
                camera_name="front",
            )
        )
    ]
    if not approved:
        return None

    if is_view_relative_stage(stage):
        # RGB prebinding has no metric range. Use perspective proximity
        # (lower box edge, then occupied area) to approximate encounter order;
        # detector score remains only a semantic tie-breaker.
        image = bundle.front_image
        width, height = image.size if image is not None and hasattr(image, "size") else (1, 1)

        def perspective_rank(detection):
            bbox = list(getattr(detection, "bbox", []) or [])
            if len(bbox) < 4:
                return (0.0, 0.0, 0.0)
            box_width = max(0.0, float(bbox[2]) - float(bbox[0]))
            box_height = max(0.0, float(bbox[3]) - float(bbox[1]))
            return (
                min(1.0, float(bbox[3]) / max(float(height), 1.0)),
                (box_width * box_height) / max(float(width * height), 1.0),
                _detection_reliability(stage, detection, image),
            )

        ordered = sorted(approved, key=perspective_rank, reverse=True)
        ordinal = max(1, int(getattr(stage, "ordinal", None) or 1))
        return ordered[ordinal - 1] if ordinal <= len(ordered) else None

    preferred = getattr(bundle, "front_detection", None)
    if preferred is not None and any(preferred is candidate for candidate in approved):
        return preferred
    return max(
        approved,
        key=lambda detection: _detection_reliability(stage, detection, bundle.front_image),
    )


def _primary_update_event(memory, stage, events):
    primary = memory.primary_instance(stage) if memory is not None else None
    if primary is None:
        return None
    candidates = [
        event
        for event in list(events or [])
        if str(getattr(event, "instance_id", "") or "") == str(primary.instance_id)
    ]
    if not candidates:
        return None
    return max(
        candidates,
        key=lambda event: (
            float(getattr(event, "reliability", 0.0) or 0.0),
            float(getattr(event, "score", 0.0) or 0.0),
        ),
    )


def _prebind_target_from_bundle(
    objects,
    stage,
    bundle: DetectionDepthBundle | None,
    *,
    source: str,
) -> bool:
    """Prebind one RGB bearing and optionally save its first boxed frame.

    Prebinding deliberately strips every depth field before updating the
    bearing tracker.  It therefore cannot create or move a MissionMemory
    instance; reliable depth observations still own the formal surface lock.
    """

    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    if not bool(config.get("PREBIND_ENABLED", True)) or bundle is None:
        return False
    if getattr(stage, "mode", "") not in {"target", "detect"}:
        return False
    if (
        bundle.front_image is None
        or bundle.observer_world is None
        or bundle.observer_yaw_deg is None
    ):
        return False

    detection = _select_prebind_front_detection(objects, stage, bundle)
    if detection is None:
        return False
    tracker = getattr(objects, "target_bearing_tracker", None)
    if tracker is None:
        return False

    rgb_detection = SimpleNamespace(
        visible=bool(getattr(detection, "visible", False)),
        bbox=list(getattr(detection, "bbox", None) or []),
        score=float(getattr(detection, "score", 0.0) or 0.0),
        camera="front",
        depth_median=None,
        depth_valid_ratio=None,
        depth_mad_m=None,
    )
    observation = tracker.record(
        stage_key=_runtime_stage_key(stage),
        detection=rgb_detection,
        image=bundle.front_image,
        observer_yaw_deg=float(bundle.observer_yaw_deg),
        source=source,
    )
    if observation is None:
        return False

    target_name = str(
        getattr(stage, "target_query", None)
        or getattr(stage, "target", None)
        or getattr(detection, "label", None)
        or "target"
    )
    print(
        "  [TargetPrebind] "
        f"target={target_name!r} angle={observation.relative_angle_deg:+.1f}deg "
        f"score={observation.score:.2f} source={source}"
    )

    recorder = getattr(objects, "target_snapshot_recorder", None)
    if recorder is not None and bool(getattr(recorder, "enabled", False)):
        stage_index = getattr(stage, "index", None)
        try:
            normalized_stage_index = int(stage_index) if stage_index is not None else None
        except (TypeError, ValueError):
            normalized_stage_index = None
        destination = recorder.record_first(
            stage_key=_runtime_stage_key(stage),
            stage_index=normalized_stage_index,
            instance_id="rgb_prebind",
            target_name=target_name,
            view="front",
            image=bundle.front_image,
            detection=detection,
            observer_world=bundle.observer_world,
            observer_yaw_deg=bundle.observer_yaw_deg,
            source=source,
            record_kind="prebind",
        )
        if destination is not None:
            print(
                "  [TargetSnapshot] "
                f"saved={destination} target={target_name!r} kind=prebind view=front"
            )
    return True


def _record_locked_target_snapshot(
    objects,
    stage,
    bundle: DetectionDepthBundle | None,
    *,
    source: str,
    events: list | tuple | None = None,
) -> None:
    """Save the exact detector observation assigned to the locked instance.

    Never re-select a high-score box from the frame here. The memory update
    event is the authoritative detection-to-instance association.
    """

    recorder = getattr(objects, "target_snapshot_recorder", None)
    memory = getattr(objects, "mission_memory", None)
    if (
        recorder is None
        or not bool(getattr(recorder, "enabled", False))
        or memory is None
        or bundle is None
    ):
        return
    is_locked = getattr(memory, "is_primary_locked", None)
    if not callable(is_locked) or not is_locked(stage):
        return
    primary = memory.primary_instance(stage)
    if primary is None:
        return

    exact_events = [
        event
        for event in list(events or [])
        if str(getattr(event, "instance_id", "") or "") == str(primary.instance_id)
        and getattr(event, "image", None) is not None
        and bool(getattr(getattr(event, "detection", None), "visible", False))
    ]
    if not exact_events:
        _debug_print(
            "  [TargetSnapshot] locked snapshot deferred: "
            f"instance={primary.instance_id} has no exact observation in this update"
        )
        return
    exact_event = max(
        exact_events,
        key=lambda event: (
            float(getattr(event, "reliability", 0.0) or 0.0),
            float(getattr(event, "score", 0.0) or 0.0),
        ),
    )
    view_name = str(getattr(exact_event, "view", "front") or "front")
    image = exact_event.image
    detection = exact_event.detection
    target_name = str(
        getattr(stage, "target_query", None)
        or getattr(stage, "target", None)
        or getattr(detection, "label", None)
        or "target"
    )
    stage_index = getattr(stage, "index", None)
    try:
        normalized_stage_index = int(stage_index) if stage_index is not None else None
    except (TypeError, ValueError):
        normalized_stage_index = None
    destination = recorder.record_first(
        stage_key=CompletionPipeline.stage_key(stage),
        stage_index=normalized_stage_index,
        instance_id=str(getattr(primary, "instance_id", "") or ""),
        target_name=target_name,
        view=view_name,
        image=image,
        detection=detection,
        observer_world=(
            list(getattr(exact_event, "observer_world", []) or [])
            or bundle.observer_world
        ),
        observer_yaw_deg=(
            getattr(exact_event, "observer_yaw_deg", None)
            if getattr(exact_event, "observer_yaw_deg", None) is not None
            else bundle.observer_yaw_deg
        ),
        source=source,
        record_kind="locked_final",
        metadata={
            "authoritative": True,
            "association": "exact_memory_update_event",
            "detection_world": [float(v) for v in list(getattr(exact_event, "world", []) or [])[:3]],
            "final_target_world": [float(v) for v in list(getattr(primary, "target_world", []) or [])[:3]],
            "memory_confidence": float(getattr(primary, "confidence", 0.0) or 0.0),
            "memory_uncertainty_m": float(getattr(primary, "uncertainty_m", 0.0) or 0.0),
            "reliability": float(getattr(exact_event, "reliability", 0.0) or 0.0),
            "selection_rule": str(getattr(stage, "selection_rule", "") or "stable"),
            "ordinal": getattr(stage, "ordinal", None),
        },
    )
    if destination is not None:
        print(
            "  [TargetSnapshot] "
            f"saved={destination} target={target_name!r} "
            f"instance={getattr(primary, 'instance_id', '')} kind=locked_final "
            f"view={view_name} association=exact_memory_update_event"
        )


def _record_target_bearing(
    objects,
    stage,
    bundle,
    detection=None,
    *,
    source: str = "active_observation",
):
    tracker = getattr(objects, "target_bearing_tracker", None)
    if tracker is None or bundle is None:
        return None
    detection = detection or _select_identity_approved_front_detection(objects, stage, bundle)
    if detection is None:
        detection = _select_identity_approved_down_detection(objects, stage, bundle)
    if detection is None or bundle.observer_world is None or bundle.observer_yaw_deg is None:
        return None
    detection_frame = getattr(detection, "camera_frame", None)
    down_frame_context = getattr(bundle.down_image, "camera_frame", None)
    bearing_image = (
        bundle.down_image
        if detection_frame is not None and detection_frame is down_frame_context
        else bundle.down_image
        if str(getattr(detection, "camera", "") or "").lower().startswith("down")
        else bundle.front_image
    )
    observation = tracker.record(
        stage_key=_runtime_stage_key(stage),
        detection=detection,
        image=bearing_image,
        observer_yaw_deg=float(bundle.observer_yaw_deg),
        source=source,
    )
    if observation is not None:
        print(
            "  [TargetBearing] "
            f"angle={observation.relative_angle_deg:+.1f}deg "
            f"score={observation.score:.2f} depth_state={observation.depth_state} "
            f"range_hint={('none' if observation.range_hint_m is None else f'{observation.range_hint_m:.1f}m')} "
            f"source={observation.source}"
        )
    return observation


def _memory_estimate_is_trustworthy(objects, stage, current_world) -> bool:
    memory = getattr(objects, "mission_memory", None)
    if memory is None:
        return False
    estimate = memory.estimate_distance(stage, current_world)
    if estimate is None:
        return False
    config = getattr(memory, "config", {}) or {}
    return bool(
        float(estimate.get("confidence", 0.0) or 0.0)
        >= float(config.get("MEMORY_DISTANCE_MIN_CONFIDENCE", 0.45))
        and float(estimate.get("uncertainty_m", 999.0) or 999.0)
        <= float(config.get("BEARING_MEMORY_MAX_UNCERTAINTY_M", 5.0))
        and float(estimate.get("observation_age_s", 999.0) or 999.0)
        <= float(config.get("BEARING_MEMORY_MAX_AGE_S", 30.0))
    )


def _bearing_only_active(objects, stage, current_world) -> bool:
    tracker = getattr(objects, "target_bearing_tracker", None)
    activation = (
        tracker.activation_guard(_runtime_stage_key(stage))
        if tracker is not None and hasattr(tracker, "activation_guard")
        else None
    )
    if activation is not None:
        # The first accepted leg must still respect the activation-view RGB
        # direction even if the same frame also produced a usable metric lock.
        return True
    observation = tracker.current(_runtime_stage_key(stage)) if tracker is not None else None
    if observation is None:
        return False
    if "prebind" in str(getattr(observation, "source", "")).lower():
        # The activation frame is newer identity evidence than any bootstrap
        # anchor.  Keep it in charge of the first leg until a reliable metric
        # observation upgrades the tracker.
        return True
    if observation.depth_state == "metric":
        return False
    return not _memory_estimate_is_trustworthy(objects, stage, current_world)


def _detection_matches_view_relative_sector(
    stage: Any,
    detection: Any,
    image: Any,
    *,
    camera_name: str,
) -> bool:
    if str(camera_name or "").lower() != "front":
        return False
    bbox = list(getattr(detection, "bbox", []) or [])
    if len(bbox) < 4 or image is None or not hasattr(image, "size"):
        return False
    width = float(image.size[0])
    if width <= 1.0:
        return False
    center_ratio = (float(bbox[0]) + float(bbox[2])) / (2.0 * width)
    text = " ".join((
        str(getattr(stage, "instruction", "") or ""),
        str(getattr(stage, "completion_condition", "") or ""),
    )).lower().replace("-", " ")
    if any(token in text for token in ("front right", "right front", "on the right", "right side", "右前方", "右侧", "右边")):
        return center_ratio > 0.5
    if any(token in text for token in ("front left", "left front", "on the left", "left side", "左前方", "左侧", "左边")):
        return center_ratio < 0.5
    return True
