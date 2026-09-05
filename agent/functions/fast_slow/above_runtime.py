"""Above-task navigation state, roof acquisition, and vertical guards.

This module contains the above-specific geometry and AirSim command
quantization helpers. Queue execution and the main runtime loop remain in
runtime.py; the exported names are re-imported there for compatibility.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from config import cfg

from agent.functions.fast_slow.completion_pipeline import CompletionPipeline
from agent.functions.fast_slow.runtime_context import RuntimeObjects
from agent.functions.memory.mission_memory import stage_key as _mission_stage_key
from agent.functions.memory.spatial_reasoning import relation_kind
from agent.functions.perception import estimate_down_roof_plane


@dataclass
class AboveStageRuntimeState:
    entry_z: float
    last_lock_id: str = ""
    last_anchor_world: list[float] | None = None
    pending_queue_review: str = ""
    require_roof_before_next_completion: bool = False
    completion_resume_pose: list[float] | None = None
    roof_acquire_origin_world: list[float] | None = None
    roof_acquire_direction_world: list[float] | None = None
    roof_acquire_goal_distance_m: float = 0.0
    # Before a roof plane is visible, front-depth facade samples provide a
    # conservative vertical envelope.  Climb legs are executed without XY
    # motion, then one newer facade observation is required before crossing.
    pre_roof_climb_target_z: float | None = None
    pre_roof_climb_surface_observations: int = 0
    pre_roof_waiting_reobserve: bool = False
    last_facade_clearance_z: float | None = None
    # Reorientation is a bounded recovery action.  Keeping the counter with
    # the stage (rather than the transient planner job) prevents a rejected
    # Qwen plan from causing an endless turn/replan loop.
    reorient_attempts: int = 0
    last_reorient_at: float = 0.0
    small_down_center_confirmations: int = 0
    last_small_down_capture_id: str = ""


def _is_above_stage(stage: Any) -> bool:
    return relation_kind(stage) == "above"


_SMALL_ABOVE_TARGET_TOKENS = (
    "car", "vehicle", "truck", "bus", "van", "person", "human", "fountain",
    "statue", "bench", "chair", "cone", "汽车", "车辆", "卡车", "公交车",
    "面包车", "人物", "人", "喷泉", "雕像", "长椅", "椅子", "路锥",
)
_LARGE_ABOVE_TARGET_TOKENS = (
    "building", "tower", "skyscraper", "warehouse", "hangar", "factory",
    "apartment", "office", "建筑", "大厦", "高楼", "塔", "仓库", "厂房",
)


def _explicit_small_above_target(stage: Any) -> bool:
    identity_parts = [
        str(getattr(stage, "target", "") or ""),
        str(getattr(stage, "target_query", "") or ""),
    ]
    # Parser fallbacks occasionally leave target fields empty; only then use
    # the instruction as an identity hint so an anchor mentioned in a
    # building instruction cannot accidentally disable roof verification.
    identity_text = " ".join(part for part in identity_parts if part.strip())
    text = (identity_text or str(getattr(stage, "instruction", "") or "")).lower()
    if any(token in text for token in _LARGE_ABOVE_TARGET_TOKENS):
        return False
    return any(token in text for token in _SMALL_ABOVE_TARGET_TOKENS)


def _small_above_target(objects: RuntimeObjects, stage: Any):
    if not _is_above_stage(stage) or not _explicit_small_above_target(stage):
        return None
    memory = getattr(objects, "mission_memory", None)
    primary_fn = getattr(memory, "primary_instance", None) if memory is not None else None
    instance = primary_fn(stage) if callable(primary_fn) else None
    if instance is None or bool(getattr(instance, "is_large_structure", False)):
        return None
    return instance


def _small_down_detection_is_centered(bundle: Any, config: dict) -> bool:
    detection = getattr(bundle, "down_detection", None)
    if detection is None:
        detection = next(
            (
                candidate
                for candidate in list(getattr(bundle, "down_detections", []) or [])
                if candidate is not None and bool(getattr(candidate, "visible", False))
            ),
            None,
        )
    image = getattr(bundle, "down_image", None)
    if detection is None or not bool(getattr(detection, "visible", False)):
        return False
    bbox = list(getattr(detection, "bbox", []) or [])
    size = getattr(image, "size", None)
    if len(bbox) < 4 or not size or float(size[0]) <= 1.0 or float(size[1]) <= 1.0:
        return False
    center_x = ((float(bbox[0]) + float(bbox[2])) * 0.5) / float(size[0])
    center_y = ((float(bbox[1]) + float(bbox[3])) * 0.5) / float(size[1])
    tolerance = max(0.05, float(config.get("ABOVE_DOWN_CENTER_TOLERANCE_RATIO", 0.22)))
    return abs(center_x - 0.5) <= tolerance and abs(center_y - 0.5) <= tolerance


def _complete_small_above_stage(objects: RuntimeObjects, path_stream, state, stage, reason: str) -> bool:
    """Complete a small-target above stage without invoking roof geometry/VLM."""
    memory = getattr(objects, "mission_memory", None)
    if path_stream is not None:
        emergency_stop = getattr(path_stream, "emergency_stop", None)
        if callable(emergency_stop):
            emergency_stop()
        else:
            path_stream.stop()
    controller = getattr(objects, "controller", None)
    clear = getattr(controller, "clear", None) if controller is not None else None
    if callable(clear):
        clear()
    pipeline = getattr(objects, "completion_pipeline", None)
    if pipeline is not None and callable(getattr(pipeline, "clear", None)):
        pipeline.clear()
    if memory is not None and callable(getattr(memory, "archive_stage", None)):
        memory.archive_stage(stage, reason)
    if state is not None:
        state.update(trajectory_queue=[], qwen_waypoints=[], status="running")
        if memory is not None and callable(getattr(memory, "summary", None)):
            state.update(memory_summary=memory.summary(stage))
    manager = getattr(objects, "task_manager", None)
    if manager is None or not callable(getattr(manager, "complete_current", None)):
        return False
    manager.complete_current(reason)
    stage_key_fn = getattr(CompletionPipeline, "stage_key", None)
    completed_key = stage_key_fn(stage) if callable(stage_key_fn) else None
    if completed_key is not None:
        getattr(objects, "completion_attempts", {}).pop(completed_key, None)
        getattr(objects, "completion_retry_after", {}).pop(completed_key, None)
    metrics = getattr(objects, "navigation_metrics", None)
    if metrics is not None:
        metrics.task_completed = bool(callable(getattr(manager, "is_done", None)) and manager.is_done())
    if callable(getattr(manager, "summary", None)):
        print(f"  [TASK] {manager.summary()}")
    if metrics is not None and bool(getattr(metrics, "task_completed", False)) and state is not None:
        state.update(status="done", task_done=True, step=0)
    return bool(getattr(metrics, "task_completed", False))


def _handle_small_above_down_center_observation(objects, path_stream, state, stage, bundle):
    """Use two distinct centered down-view detections for small ``above`` targets.

    The first hit stops the active path; the second fresh capture completes the
    stage.  This gives fountains/cars a deterministic local completion path
    while leaving large buildings on the roof-plane pipeline.
    """
    instance = _small_above_target(objects, stage)
    if instance is None:
        return False, False
    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    if not _small_down_detection_is_centered(bundle, config):
        return False, False
    capture_id = str(getattr(bundle, "capture_id", "") or "")
    current_world = list(getattr(bundle, "observer_world", None) or [])
    target_world = list(getattr(instance, "target_world", None) or [])
    if len(current_world) >= 2 and len(target_world) >= 2:
        center_tolerance = max(
            1.0,
            float(getattr(instance, "footprint_radius_m", 1.5) or 1.5)
            * float(config.get("ABOVE_DOWN_CENTER_TOLERANCE_RATIO", 0.22))
            + min(float(getattr(instance, "uncertainty_m", 0.0) or 0.0), 2.0),
        )
        if math.hypot(
            float(current_world[0]) - float(target_world[0]),
            float(current_world[1]) - float(target_world[1]),
        ) > center_tolerance:
            return False, False
    runtime_state = _above_stage_state(objects, stage, current_world or [0.0, 0.0, 0.0])
    if capture_id and capture_id == runtime_state.last_small_down_capture_id:
        return True, False
    runtime_state.last_small_down_capture_id = capture_id
    runtime_state.small_down_center_confirmations += 1
    required_confirmations = max(1, int(config.get("ABOVE_DOWN_CENTER_CONFIRMATIONS", 2)))
    if runtime_state.small_down_center_confirmations < required_confirmations:
        if path_stream is not None:
            emergency_stop = getattr(path_stream, "emergency_stop", None)
            if callable(emergency_stop):
                emergency_stop()
            else:
                path_stream.stop()
        clear = getattr(getattr(objects, "controller", None), "clear", None)
        if callable(clear):
            clear()
        print(
            "  [AboveSmall] down-view target centered; stopped for fresh confirmation "
            f"{runtime_state.small_down_center_confirmations}/{required_confirmations}"
        )
        return True, False
    reason = "small above target centered in two fresh down views"
    done = _complete_small_above_stage(objects, path_stream, state, stage, reason)
    return True, done


def _small_above_confirmation_pending(objects: RuntimeObjects, stage: Any) -> bool:
    """Whether a first centered hit is waiting for its next fresh frame."""
    instance = _small_above_target(objects, stage)
    if instance is None:
        return False
    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    required = max(1, int(config.get("ABOVE_DOWN_CENTER_CONFIRMATIONS", 2)))
    runtime_state = _above_stage_state(objects, stage, [0.0, 0.0, 0.0])
    return 0 < runtime_state.small_down_center_confirmations < required


def _apply_small_above_target_guidance(
    objects: RuntimeObjects,
    stage: Any,
    cumulative_waypoints: list,
    memory_context: dict,
    *,
    selection_pos,
    planning_wall_s: float = 0.0,
) -> tuple[list, str]:
    """Replace a forward-only small-target path with a smooth target curve.

    Small ``above`` targets do not need a roof probe.  If the selected Qwen
    endpoint misses the locked target's XY bearing, use the available planning
    horizon to create a short, evenly spaced body-frame curve.  The last point
    reaches the target when it is inside that horizon; otherwise it remains a
    convergent prefix and the next RGB/depth capture extends it.
    """
    instance = _small_above_target(objects, stage)
    if instance is None:
        return cumulative_waypoints, ""
    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    if not bool(config.get("ABOVE_TARGET_GUIDANCE_ENABLED", True)):
        return cumulative_waypoints, ""
    target = list((memory_context or {}).get("target_body") or [])
    if len(target) < 2:
        return cumulative_waypoints, ""
    target_xy = [float(target[0]), float(target[1])]
    target_distance = math.hypot(target_xy[0], target_xy[1])
    if target_distance <= 1e-6:
        return cumulative_waypoints, ""
    endpoint = list(cumulative_waypoints[-1]) if cumulative_waypoints else [0.0, 0.0, 0.0]
    endpoint_distance = math.hypot(float(endpoint[0]), float(endpoint[1])) if len(endpoint) >= 2 else 0.0
    target_angle = math.degrees(math.atan2(target_xy[1], target_xy[0]))
    endpoint_angle = math.degrees(math.atan2(float(endpoint[1]), float(endpoint[0]))) if endpoint_distance > 1e-6 else 0.0
    deviation = abs((endpoint_angle - target_angle + 180.0) % 360.0 - 180.0)
    projection = float(endpoint[0]) * target_xy[0] / target_distance + float(endpoint[1]) * target_xy[1] / target_distance
    remaining = math.hypot(target_xy[0] - float(endpoint[0]), target_xy[1] - float(endpoint[1]))
    min_progress_ratio = max(0.0, float(config.get("ABOVE_TARGET_GUIDANCE_MIN_PROGRESS_RATIO", 0.15)))
    max_deviation = float(config.get("ABOVE_TARGET_GUIDANCE_MAX_DEVIATION_DEG", 15.0))
    if (
        cumulative_waypoints
        and deviation <= max_deviation
        and projection >= max(0.5, target_distance * min_progress_ratio)
        and remaining <= target_distance * (1.0 - min_progress_ratio)
    ):
        return cumulative_waypoints, ""
    nominal_speed = max(0.1, float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0)))
    fallback_s = max(0.1, float(config.get("ABOVE_CONTINUOUS_FALLBACK_PLANNING_S", 5.5)))
    margin_s = max(0.0, float(config.get("ABOVE_CONTINUOUS_HORIZON_MARGIN_S", 1.5)))
    horizon = max(
        1.0,
        float(config.get("ABOVE_TARGET_GUIDANCE_MIN_HORIZON_M", 12.0)),
        nominal_speed * max(fallback_s, float(planning_wall_s) + margin_s),
    )
    controller = getattr(objects, "controller", None)
    horizon_fn = getattr(controller, "required_planning_horizon_m", None)
    if callable(horizon_fn):
        try:
            horizon = max(horizon, float(horizon_fn(nominal_speed)))
        except (TypeError, ValueError):
            pass
    distance = min(target_distance, horizon)
    unit = [target_xy[0] / target_distance, target_xy[1] / target_distance]
    spacing = max(0.5, float(config.get("ABOVE_TARGET_GUIDANCE_SPACING_M", 3.0)))
    max_points = max(3, int(config.get("ABOVE_TARGET_GUIDANCE_MAX_WAYPOINTS", 6)))
    count = max(3, min(max_points, int(math.ceil(distance / spacing))))
    guided = [
        [
            round(unit[0] * distance * index / count, 3),
            round(unit[1] * distance * index / count, 3),
            0.0,
        ]
        for index in range(1, count + 1)
    ]
    return guided, f"smooth_locked_target_{deviation:.1f}deg_horizon_{distance:.1f}m"


def _above_stage_state(objects: RuntimeObjects, stage: Any, current_world) -> AboveStageRuntimeState:
    key = CompletionPipeline.stage_key(stage)
    states = getattr(objects, "above_stage_states", None)
    if states is None:
        states = {}
        setattr(objects, "above_stage_states", states)
    state = states.get(key)
    if state is None:
        memory = getattr(objects, "mission_memory", None)
        primary = memory.primary_instance(stage) if memory is not None else None
        stage_locks = getattr(memory, "stage_locks", {}) if memory is not None else {}
        lock_id = (
            str(stage_locks.get(_mission_stage_key(stage), "") or "")
            if isinstance(stage_locks, dict)
            else ""
        )
        state = AboveStageRuntimeState(
            entry_z=float(current_world[2]),
            last_lock_id=lock_id,
            last_anchor_world=list(primary.target_world) if primary is not None else None,
        )
        states[key] = state
    return state


def _locked_large_above_requires_roof(objects: RuntimeObjects, stage: Any) -> bool:
    """Whether a facade point is forbidden from completing this ``above`` stage."""
    if not _is_above_stage(stage):
        return False
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not bool(getattr(memory, "config", {}).get("ABOVE_REQUIRE_ROOF_GEOMETRY", True)):
        return False
    # Relation ``above`` is also used for cars, fountains and people.  Their
    # completion contract is target-centered and must never be promoted to a
    # building roof check merely because a detector produced a large box.
    if _explicit_small_above_target(stage):
        return False
    instance = memory.primary_instance(stage)
    locked_fn = getattr(memory, "is_primary_locked", None)
    return bool(
        instance is not None
        and callable(locked_fn)
        and locked_fn(stage)
        and bool(getattr(instance, "is_large_structure", False))
    )


def _above_roof_candidate_ready(
    objects: RuntimeObjects,
    stage: Any,
    current_world,
    *,
    require_trusted: bool = False,
) -> bool:
    """Use one roof gate for the runtime, watchdog, idle trigger and queue hold.

    One retained (but not yet trusted) roof plane may launch a synchronized
    completion capture, which can provide the second consistent observation.
    A facade-only point may never launch that capture.
    """
    if not _locked_large_above_requires_roof(objects, stage):
        return True
    memory = getattr(objects, "mission_memory", None)
    roof = memory.roof_navigation_context(stage, current_world) if memory is not None else None
    if roof is None:
        return False
    if require_trusted and not bool(roof.get("trusted", False)):
        return False
    return True


def _cumulative_path_length(cumulative_waypoints: list) -> float:
    previous = [0.0, 0.0, 0.0]
    total = 0.0
    for waypoint in list(cumulative_waypoints or []):
        if waypoint is None or len(waypoint) < 3:
            continue
        current = [float(value) for value in waypoint[:3]]
        total += math.sqrt(sum((current[index] - previous[index]) ** 2 for index in range(3)))
        previous = current
    return total


def _straight_body_path(direction_body_xy, distance_m: float, spacing_m: float, max_points: int) -> list[list[float]]:
    distance = max(0.0, float(distance_m))
    norm = math.hypot(float(direction_body_xy[0]), float(direction_body_xy[1]))
    if distance <= 1e-6 or norm <= 1e-6:
        return []
    unit = [float(direction_body_xy[0]) / norm, float(direction_body_xy[1]) / norm]
    count = max(1, min(max(1, int(max_points)), int(math.ceil(distance / max(0.5, spacing_m)))))
    return [
        [
            round(unit[0] * distance * index / count, 3),
            round(unit[1] * distance * index / count, 3),
            0.0,
        ]
        for index in range(1, count + 1)
    ]


def _apply_above_roof_acquisition_path_guard(
    objects: RuntimeObjects,
    stage: Any,
    cumulative_waypoints: list,
    *,
    selection_pos,
    selection_yaw: float,
    planning_wall_s: float,
) -> tuple[list, str]:
    """Keep an altitude-held XY path alive until down depth finds the roof.

    Close-range Qwen plans can be shorter than one model call.  For a locked
    large building with no roof geometry, replace only those short plans with
    a bounded memory-directed horizon.  Once close to the facade anchor, keep
    the original approach direction and probe a short distance through the
    anchor so the nadir camera can enter the roof footprint.
    """
    if not _locked_large_above_requires_roof(objects, stage):
        return cumulative_waypoints, ""
    memory = getattr(objects, "mission_memory", None)
    current = [float(value) for value in list(selection_pos or [])[:3]]
    if memory is None or len(current) < 3:
        return cumulative_waypoints, ""
    roof = memory.roof_navigation_context(stage, current)
    runtime_state = _above_stage_state(objects, stage, current)
    if roof is not None and (
        bool(roof.get("trusted", False))
        or not runtime_state.require_roof_before_next_completion
    ):
        # Even one retained plane is enough to stop blind facade probing.  The
        # synchronized completion capture can now confirm the plane.
        if bool(roof.get("trusted", False)):
            runtime_state.roof_acquire_origin_world = None
            runtime_state.roof_acquire_direction_world = None
            runtime_state.roof_acquire_goal_distance_m = 0.0
        return cumulative_waypoints, ""

    instance = memory.primary_instance(stage)
    target = list(getattr(instance, "target_world", None) or [])
    if len(target) < 2:
        return cumulative_waypoints, ""
    dx = float(target[0]) - current[0]
    dy = float(target[1]) - current[1]
    horizontal = math.hypot(dx, dy)
    memory_cfg = getattr(memory, "config", {}) or {}
    overhead = memory.above_overhead_context(stage, current) or {}
    acquisition_radius = max(
        1.0,
        float(overhead.get("trigger_radius_m", 0.0) or 0.0),
        float(getattr(instance, "footprint_radius_m", 1.5) or 1.5)
        + float(memory_cfg.get("ABOVE_HORIZONTAL_RADIUS_M", 3.5)),
    )

    nominal_speed = max(0.1, float(cfg.get("SIM", {}).get("AIRSIM_VELOCITY", 2.0)))
    fallback_planning_s = max(0.1, float(memory_cfg.get("ABOVE_CONTINUOUS_FALLBACK_PLANNING_S", 5.5)))
    margin_s = max(0.0, float(memory_cfg.get("ABOVE_CONTINUOUS_HORIZON_MARGIN_S", 1.5)))
    horizon_s = max(fallback_planning_s, max(0.0, float(planning_wall_s)) + margin_s)
    minimum_horizon = max(
        1.0,
        float(memory_cfg.get("ABOVE_MIN_CONTINUOUS_HORIZON_M", 12.0)),
        nominal_speed * horizon_s,
    )
    max_leg = max(1.0, float(memory_cfg.get("PATH_MAX_GUIDED_LEG_M", 18.0)))
    minimum_horizon = min(minimum_horizon, max_leg)

    phase = "approach"
    direction_world = None
    desired_distance = minimum_horizon
    if horizontal <= acquisition_radius:
        phase = "roof_acquire"
        if runtime_state.roof_acquire_direction_world is None:
            if horizontal > 1e-6:
                direction_world = [dx / horizontal, dy / horizontal]
            else:
                yaw_rad = math.radians(float(selection_yaw))
                direction_world = [math.cos(yaw_rad), math.sin(yaw_rad)]
            runtime_state.roof_acquire_origin_world = list(current)
            runtime_state.roof_acquire_direction_world = list(direction_world)
            beyond_anchor = max(1.0, float(memory_cfg.get("ABOVE_ROOF_PROBE_BEYOND_ANCHOR_M", 8.0)))
            max_probe = max(beyond_anchor, float(memory_cfg.get("ABOVE_ROOF_PROBE_MAX_TRAVEL_M", 24.0)))
            runtime_state.roof_acquire_goal_distance_m = min(
                max_probe,
                max(minimum_horizon, horizontal + beyond_anchor),
            )
        direction_world = list(runtime_state.roof_acquire_direction_world or [])
        origin = list(runtime_state.roof_acquire_origin_world or current)
        progress = max(
            0.0,
            (current[0] - float(origin[0])) * float(direction_world[0])
            + (current[1] - float(origin[1])) * float(direction_world[1]),
        )
        remaining = max(0.0, float(runtime_state.roof_acquire_goal_distance_m) - progress)
        if remaining <= 0.5:
            return cumulative_waypoints, "roof_probe_exhausted_planner_control"
        desired_distance = min(max_leg, remaining)
    elif horizontal > 1e-6:
        direction_world = [dx / horizontal, dy / horizontal]

    if not direction_world or len(direction_world) < 2:
        return cumulative_waypoints, ""
    existing_length = _cumulative_path_length(cumulative_waypoints)
    yaw_rad = math.radians(float(selection_yaw))
    body_direction = [
        math.cos(yaw_rad) * float(direction_world[0]) + math.sin(yaw_rad) * float(direction_world[1]),
        -math.sin(yaw_rad) * float(direction_world[0]) + math.cos(yaw_rad) * float(direction_world[1]),
    ]
    endpoint = list(cumulative_waypoints[-1]) if cumulative_waypoints else [0.0, 0.0, 0.0]
    endpoint_projection = (
        float(endpoint[0]) * body_direction[0] + float(endpoint[1]) * body_direction[1]
        if len(endpoint) >= 2
        else 0.0
    )
    if existing_length >= desired_distance * 0.95 and endpoint_projection >= desired_distance * 0.75:
        return cumulative_waypoints, ""

    spacing = max(0.5, float(memory_cfg.get("ABOVE_ROOF_ACQUIRE_WAYPOINT_SPACING_M", 3.0)))
    max_points = max(2, int(memory_cfg.get("ABOVE_ROOF_ACQUIRE_MAX_WAYPOINTS", 8)))
    guarded = _straight_body_path(body_direction, desired_distance, spacing, max_points)
    reason = (
        f"{phase}_continuous_horizon_{desired_distance:.1f}m "
        f"planner={float(planning_wall_s):.1f}s anchor={horizontal:.1f}m"
    )
    return guarded, reason


def _above_pre_roof_facade_clearance_context(
    objects: RuntimeObjects,
    stage: Any,
    current_world,
) -> dict:
    """Estimate a conservative climb envelope from locked front-facade depth.

    A horizontal front camera cannot verify a roof plane while the UAV is
    below it, but the uppermost retained facade samples still prove that the
    building occupies that altitude.  NED Z decreases while climbing, so a
    safe pre-roof target is above (more negative than) that highest sample.
    """

    if not _locked_large_above_requires_roof(objects, stage):
        return {}
    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    if not bool(config.get("ABOVE_PRE_ROOF_FACADE_CLIMB_ENABLED", True)):
        return {}
    current = [float(value) for value in list(current_world or [])[:3]]
    if memory is None or len(current) < 3:
        return {}
    roof = memory.roof_navigation_context(stage, current)
    if bool((roof or {}).get("trusted", False)):
        return {}
    instance = memory.primary_instance(stage)
    if instance is None:
        return {}

    facade_z_values = []
    bounds = list(getattr(instance, "surface_bounds_world", None) or [])
    if bounds and len(bounds[0]) >= 3:
        facade_z_values.append(float(bounds[0][2]))
    for point in list(getattr(instance, "surface_points_world", None) or []):
        if point is not None and len(point) >= 3:
            facade_z_values.append(float(point[2]))
    for patch in list(getattr(instance, "surface_patches_world", None) or []):
        for point in list(patch or []):
            if point is not None and len(point) >= 3:
                facade_z_values.append(float(point[2]))
    facade_z_values = [value for value in facade_z_values if math.isfinite(value)]
    if not facade_z_values:
        return {}

    highest_facade_z = min(facade_z_values)
    clearance_m = max(0.5, float(config.get("ABOVE_PRE_ROOF_FACADE_CLEARANCE_M", 3.0)))
    uncertainty_margin_m = max(
        0.0,
        float(config.get("ABOVE_PRE_ROOF_FACADE_UNCERTAINTY_MARGIN_M", 1.0)),
    )
    requested_z = highest_facade_z - clearance_m - uncertainty_margin_m
    source = "front_facade"
    # One untrusted down-view roof plane is not allowed to complete the stage,
    # but using its height only to request an upward move is conservative.
    if roof is not None and roof.get("roof_z_world") is not None:
        requested_z = min(
            requested_z,
            float(roof["roof_z_world"]) - clearance_m - uncertainty_margin_m,
        )
        source = "front_facade_and_roof_candidate"

    runtime_state = _above_stage_state(objects, stage, current)
    max_total_climb_m = max(
        clearance_m,
        float(config.get("ABOVE_PRE_ROOF_MAX_TOTAL_CLIMB_M", 60.0)),
    )
    minimum_world_z = float(runtime_state.entry_z) - max_total_climb_m
    target_z = max(requested_z, minimum_world_z)
    return {
        "target_world_z": float(target_z),
        "requested_world_z": float(requested_z),
        "highest_facade_world_z": float(highest_facade_z),
        "clearance_m": float(clearance_m),
        "uncertainty_margin_m": float(uncertainty_margin_m),
        "source": source,
        "limited": bool(target_z > requested_z + 1e-6),
        "surface_observations": int(getattr(instance, "surface_observation_count", 0) or 0),
    }


def _minimum_airsim_vertical_command_m(config: dict | None = None) -> float:
    """Return a vertical displacement that is strictly greater than 5 m."""

    config = config or {}
    return max(5.001, float(config.get("AIRSIM_MIN_VERTICAL_COMMAND_M", 5.5)))


def _minimum_airsim_climb_command_m(config: dict | None = None) -> float:
    """Return the configured climb minimum without weakening AirSim's limit."""

    config = config or {}
    return max(
        _minimum_airsim_vertical_command_m(config),
        float(
            config.get(
                "AIRSIM_MIN_CLIMB_COMMAND_M",
                config.get("AIRSIM_MIN_VERTICAL_COMMAND_M", 5.5),
            )
        ),
    )


def _quantized_climb_body_z(
    current_world_z: float,
    requested_world_z: float,
    *,
    config: dict,
    preferred_max_leg_m: float,
) -> float:
    """Quantize one NED climb leg so AirSim cannot silently ignore it."""

    remaining = float(current_world_z) - float(requested_world_z)
    if remaining <= 0.0:
        return 0.0
    minimum = _minimum_airsim_climb_command_m(config)
    preferred = max(minimum, float(preferred_max_leg_m))
    magnitude = min(preferred, remaining)
    if magnitude < minimum:
        magnitude = minimum
    return -float(magnitude)


def _normalize_direct_vertical_action_m(
    value: Any,
    config: dict | None = None,
    *,
    climb: bool = False,
) -> float:
    """Preserve direction while enforcing its configured displacement minimum."""

    try:
        magnitude = abs(float(value))
    except (TypeError, ValueError):
        magnitude = 0.0
    minimum = (
        _minimum_airsim_climb_command_m(config)
        if climb
        else _minimum_airsim_vertical_command_m(config)
    )
    return max(magnitude, minimum)


def _apply_airsim_vertical_path_quantization(
    cumulative_waypoints: list,
    *,
    config: dict | None = None,
) -> tuple[list, str]:
    """Drop intermediate altitude targets below the configured minimum.

    Cumulative targets are compared with the last effective altitude. Small
    increments are held until their accumulated change reaches the applicable
    threshold (10 m for climbs by default, 5.5 m for descents). Horizontal
    motion is preserved unchanged.
    """

    if not cumulative_waypoints:
        return cumulative_waypoints, ""
    vertical_minimum = _minimum_airsim_vertical_command_m(config)
    climb_minimum = _minimum_airsim_climb_command_m(config)
    minimum = vertical_minimum
    effective_z = 0.0
    changed = 0
    guarded = []
    for waypoint in cumulative_waypoints:
        point = [float(value) for value in waypoint[:3]]
        requested_z = point[2]
        vertical_delta = requested_z - effective_z
        minimum = climb_minimum if vertical_delta < -1e-6 else vertical_minimum
        if 1e-6 < abs(vertical_delta) < minimum:
            point[2] = effective_z
            changed += 1
        elif abs(vertical_delta) >= minimum:
            effective_z = requested_z
        else:
            point[2] = effective_z
        guarded.append([round(value, 3) for value in point])
    if not changed:
        return cumulative_waypoints, ""
    return guarded, f"held_{changed}_sub_{minimum:.3f}m_vertical_targets"


def _above_max_allowed_world_z(objects: RuntimeObjects, stage: Any, current_world) -> tuple[float, str, dict]:
    """Return the deepest safe NED Z for the active above stage."""
    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    current = [float(value) for value in current_world[:3]]
    if not _is_above_stage(stage) or not bool(config.get("ABOVE_ALTITUDE_GUARD_ENABLED", True)):
        return float("inf"), "disabled", {}
    runtime_state = _above_stage_state(objects, stage, current)
    roof = memory.roof_navigation_context(stage, current) if memory is not None else None
    if not bool((roof or {}).get("trusted", False)):
        # ``above`` never needs a semantic descent. Before a roof is trusted,
        # hold the entry altitude or climb above the observed facade.
        hold_world_z = float(runtime_state.entry_z)
        facade = _above_pre_roof_facade_clearance_context(objects, stage, current)
        if facade:
            return (
                min(hold_world_z, float(facade["target_world_z"])),
                "roof_unknown_climb_above_observed_facade",
                facade,
            )
        return (
            hold_world_z,
            "roof_unknown_hold_entry_altitude",
            roof or {},
        )

    roof_z = float(roof["roof_z_world"])
    clearance = roof_z - current[2]
    min_clearance = max(0.1, float(config.get("ABOVE_MIN_CLEARANCE_M", 0.3)))
    allowed = current[2]
    mode = "roof_confirmed_hold_clearance"
    allowed = min(allowed, roof_z - min_clearance)
    return float(allowed), mode, dict(roof)


def _apply_above_altitude_path_guard(
    objects: RuntimeObjects,
    stage: Any,
    cumulative_waypoints: list,
    *,
    selection_pos,
) -> tuple[list, str]:
    if not _is_above_stage(stage):
        return cumulative_waypoints, ""
    current = [float(value) for value in list(selection_pos or [])[:3]]
    if len(current) < 3:
        return cumulative_waypoints, ""
    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    runtime_state = _above_stage_state(objects, stage, current)
    instance = memory.primary_instance(stage) if memory is not None else None
    surface_observations = int(getattr(instance, "surface_observation_count", 0) or 0)
    last_seen_view = str(getattr(instance, "last_seen_view", "") or "").strip().lower()
    last_optical_axis = [
        float(value)
        for value in list(getattr(instance, "last_seen_optical_axis_world", []) or [])[:3]
    ]
    roof = memory.roof_navigation_context(stage, current) if memory is not None else None
    roof_trusted = bool((roof or {}).get("trusted", False))
    climb_tolerance = max(
        0.05,
        float(config.get("ABOVE_PRE_ROOF_CLIMB_TOLERANCE_M", 0.4)),
    )
    max_climb_leg = max(
        _minimum_airsim_climb_command_m(config),
        float(config.get("ABOVE_PRE_ROOF_MAX_CLIMB_LEG_M", 10.0)),
    )

    if roof_trusted:
        runtime_state.pre_roof_climb_target_z = None
        runtime_state.pre_roof_waiting_reobserve = False
    elif runtime_state.pre_roof_climb_target_z is not None:
        pending_target_z = float(runtime_state.pre_roof_climb_target_z)
        if current[2] > pending_target_z + climb_tolerance:
            climb_body_z = _quantized_climb_body_z(
                current[2],
                pending_target_z,
                config=config,
                preferred_max_leg_m=max_climb_leg,
            )
            next_target_z = current[2] + climb_body_z
            return (
                [[0.0, 0.0, round(climb_body_z, 3)]],
                f"vertical_first_continue_climb target_world_z={pending_target_z:.2f} "
                f"current_world_z={current[2]:.2f}",
            )
        runtime_state.pre_roof_climb_target_z = None
    if not roof_trusted and runtime_state.pre_roof_waiting_reobserve:
        has_real_optical_axis = bool(
            len(last_optical_axis) >= 3
            and all(math.isfinite(value) for value in last_optical_axis[:3])
        )
        camera_points_clearly_downward = bool(
            has_real_optical_axis
            and last_optical_axis[2] > 0.35
            and last_optical_axis[2]
            > 0.5 * math.hypot(last_optical_axis[0], last_optical_axis[1])
        )
        latest_surface_is_facade_view = bool(
            (has_real_optical_axis and not camera_points_clearly_downward)
            or (not has_real_optical_axis and last_seen_view.startswith("front"))
        )
        post_climb_front_reobserved = bool(
            surface_observations > runtime_state.pre_roof_climb_surface_observations
            and latest_surface_is_facade_view
        )
        if (
            roof is None
            and not post_climb_front_reobserved
        ):
            # Do not cross the facade using the same observation that caused
            # the climb.  The normal target-observation pipeline keeps running
            # while the queue is empty and will release this hold with a newer
            # upper-facade envelope (or a roof plane).
            return (
                [],
                "vertical_first_waiting_for_post_climb_facade_observation "
                f"surface_observations={surface_observations} last_view={last_seen_view or 'none'}",
            )
        runtime_state.pre_roof_waiting_reobserve = False

    max_world_z, mode, roof = _above_max_allowed_world_z(objects, stage, selection_pos)
    if not math.isfinite(max_world_z):
        return cumulative_waypoints, ""
    if current[2] > float(max_world_z) + climb_tolerance:
        if (
            bool((roof or {}).get("limited", False))
            and current[2] - float(max_world_z) + 1e-6
            < _minimum_airsim_climb_command_m(config)
        ):
            return (
                [],
                "pre_roof_climb_tail_blocked_by_emergency_ceiling "
                f"remaining={current[2] - float(max_world_z):.2f}m",
            )
        climb_body_z = _quantized_climb_body_z(
            current[2],
            float(max_world_z),
            config=config,
            preferred_max_leg_m=max_climb_leg,
        )
        next_target_z = current[2] + climb_body_z
        runtime_state.pre_roof_climb_target_z = float(next_target_z)
        runtime_state.pre_roof_climb_surface_observations = surface_observations
        runtime_state.pre_roof_waiting_reobserve = not bool((roof or {}).get("trusted", False))
        context_text = ""
        if mode == "roof_unknown_climb_above_observed_facade" and roof:
            context_text = (
                f" facade_top_z={float(roof.get('highest_facade_world_z', 0.0)):.2f}"
                f" clearance={float(roof.get('clearance_m', 0.0)):.1f}m"
            )
        return (
            [[0.0, 0.0, round(climb_body_z, 3)]],
            f"{mode} vertical_first target_world_z={float(max_world_z):.2f} "
            f"leg_target_z={next_target_z:.2f}{context_text}",
        )

    if (
        mode == "roof_unknown_climb_above_observed_facade"
        and bool((roof or {}).get("limited", False))
    ):
        # The configured emergency ceiling is a hard safety boundary, not a
        # license to cross a facade that still extends above it.
        return (
            [],
            "pre_roof_climb_limit_reached_horizontal_crossing_blocked "
            f"world_z={current[2]:.2f} requested_z={float(roof.get('requested_world_z', max_world_z)):.2f}",
        )

    if not cumulative_waypoints:
        return cumulative_waypoints, ""
    # Never ask an above stage to descend; excess roof clearance is valid.
    max_body_z = min(0.0, float(max_world_z) - current[2])
    guarded = []
    changed = False
    for waypoint in cumulative_waypoints:
        point = [float(value) for value in waypoint[:3]]
        if point[2] > max_body_z:
            point[2] = max_body_z
            changed = True
        guarded.append([round(value, 3) for value in point])
    if not changed:
        return cumulative_waypoints, ""
    if not roof:
        roof_text = ""
    elif roof.get("roof_z_world") is not None:
        roof_text = (
            f" roof_z={float(roof['roof_z_world']):.2f} "
            f"clearance={float(roof.get('clearance_m', 0.0)):.2f}m"
        )
    else:
        roof_text = (
            f" facade_top_z={float(roof.get('highest_facade_world_z', 0.0)):.2f} "
            f"required_z={float(roof.get('target_world_z', max_world_z)):.2f}"
        )
    return guarded, f"{mode} max_world_z={max_world_z:.2f}{roof_text}"


def _above_queue_violation_reason(objects: RuntimeObjects, stage: Any, current_world) -> str:
    queue = list(
        getattr(
            getattr(getattr(objects, "controller", None), "queue", None),
            "world_waypoints",
            [],
        )
        or []
    )
    if not queue:
        return ""
    max_world_z, _mode, _roof = _above_max_allowed_world_z(objects, stage, current_world)
    memory = getattr(objects, "mission_memory", None)
    memory_cfg = getattr(memory, "config", {}) or {}
    tolerance = float(memory_cfg.get("ABOVE_QUEUE_Z_TOLERANCE_M", 0.15))
    current = [float(value) for value in current_world[:3]]
    climb_required = bool(current[2] > max_world_z + tolerance)
    if climb_required:
        xy_tolerance = max(
            0.05,
            float(memory_cfg.get("ABOVE_VERTICAL_FIRST_XY_TOLERANCE_M", 0.35)),
        )
        if any(
            math.hypot(float(point[0]) - current[0], float(point[1]) - current[1]) > xy_tolerance
            for point in queue
            if len(point) >= 3
        ):
            return "queued_xy_motion_before_above_facade_clearance"
        if any(float(point[2]) > current[2] + tolerance for point in queue if len(point) >= 3):
            return "queued_descent_while_above_climb_required"
    elif any(float(point[2]) > max_world_z + tolerance for point in queue if len(point) >= 3):
        return "queued_descent_exceeds_above_envelope"
    estimate = memory.estimate_distance(stage, current_world) if memory is not None else None
    target = list((estimate or {}).get("target_world") or [])
    if len(target) >= 2:
        current_distance = math.hypot(
            float(current_world[0]) - float(target[0]),
            float(current_world[1]) - float(target[1]),
        )
        endpoint = queue[-1]
        endpoint_distance = math.hypot(float(endpoint[0]) - float(target[0]), float(endpoint[1]) - float(target[1]))
        margin = float(memory.config.get("ABOVE_QUEUE_DIVERGENCE_MARGIN_M", 2.0))
        above_state = _above_stage_state(objects, stage, current)
        probe_origin = list(above_state.roof_acquire_origin_world or [])
        probe_direction = list(above_state.roof_acquire_direction_world or [])
        roof = memory.roof_navigation_context(stage, current) if memory is not None else None
        probe_active = bool(
            len(probe_origin) >= 2
            and len(probe_direction) >= 2
            and float(above_state.roof_acquire_goal_distance_m) > 0.0
            and not bool((roof or {}).get("trusted", False))
        )
        if probe_active:
            direction_norm = math.hypot(float(probe_direction[0]), float(probe_direction[1]))
            if direction_norm <= 1e-6:
                return "queued_roof_probe_has_invalid_direction"
            direction = [
                float(probe_direction[0]) / direction_norm,
                float(probe_direction[1]) / direction_norm,
            ]
            progress_tolerance = max(
                0.1,
                float(memory.config.get("ABOVE_ROOF_PROBE_PROGRESS_TOLERANCE_M", 1.0)),
            )
            corridor_half_width = max(
                0.25,
                float(memory.config.get("ABOVE_ROOF_PROBE_CORRIDOR_HALF_WIDTH_M", 3.0)),
            )
            current_offset = [current[0] - float(probe_origin[0]), current[1] - float(probe_origin[1])]
            current_progress = current_offset[0] * direction[0] + current_offset[1] * direction[1]
            goal_progress = float(above_state.roof_acquire_goal_distance_m)
            last_progress = current_progress
            for point in queue:
                if point is None or len(point) < 2:
                    continue
                offset = [float(point[0]) - float(probe_origin[0]), float(point[1]) - float(probe_origin[1])]
                progress = offset[0] * direction[0] + offset[1] * direction[1]
                lateral = abs(offset[0] * direction[1] - offset[1] * direction[0])
                if progress < last_progress - progress_tolerance:
                    return "queued_roof_probe_moves_backward"
                if progress > goal_progress + progress_tolerance:
                    return "queued_roof_probe_exceeds_goal"
                if lateral > corridor_half_width:
                    return "queued_roof_probe_leaves_corridor"
                last_progress = max(last_progress, progress)
            # Moving beyond a facade point is the intended roof-acquisition
            # motion.  Once the path is inside this bounded corridor, do not
            # reinterpret increasing distance from the facade anchor as drift.
            return ""
        if endpoint_distance > current_distance + margin:
            return "queued_path_diverges_from_locked_above_target"
    return ""


def _record_synchronized_roof_plane(
    objects: RuntimeObjects,
    stage: Any,
    down_depth,
    observer_world,
    observer_yaw_deg: float,
    *,
    state=None,
    source: str,
    camera_frame=None,
    additional_camera_frames=None,
):
    memory = getattr(objects, "mission_memory", None)
    if memory is None or (down_depth is None and not additional_camera_frames) or not _is_above_stage(stage):
        return None
    primary = memory.primary_instance(stage)
    # Fountains, cars and people have a center/overhead contract.  Running a
    # roof-plane estimator for them only produces rejected diagnostics and can
    # accidentally route completion back into the large-building pipeline.
    if primary is not None and _explicit_small_above_target(stage) and not bool(
        getattr(primary, "is_large_structure", False)
    ):
        return None
    target_xy_world = list(primary.target_world[:2]) if primary is not None else None
    candidates = []
    if down_depth is not None:
        candidates.append((down_depth, camera_frame))
    for extra_frame in list(additional_camera_frames or []):
        if extra_frame is not None and getattr(extra_frame, "depth", None) is not None:
            candidates.append((extra_frame.depth, extra_frame))
    estimates = [
        estimate_down_roof_plane(
            depth_value,
            observer_world,
            observer_yaw_deg,
            config=memory.config,
            sim_config=memory.sim_config,
            camera_frame=frame_value,
            target_xy_world=target_xy_world,
        )
        for depth_value, frame_value in candidates
    ]
    valid_estimates = [value for value in estimates if value.valid]
    estimate = (
        max(valid_estimates, key=lambda value: float(value.confidence))
        if valid_estimates
        else estimates[0]
    )
    summary = estimate.to_summary_dict()
    if not estimate.valid:
        z_mad_text = "n/a" if summary["z_mad_m"] is None else f"{float(summary['z_mad_m']):.3f}m"
        normal_text = (
            "n/a"
            if summary["normal_error_deg"] is None
            else f"{float(summary['normal_error_deg']):.2f}deg"
        )
        print(
            f"  [RoofPlane] source={source} accepted=False reason={estimate.reason} "
            f"valid={summary['valid_ratio']:.3f} coverage={summary['coverage_ratio']:.3f} "
            f"center={summary['center_support_ratio']:.3f} z_mad={z_mad_text} normal={normal_text}"
        )
        return estimate
    previous_event = dict(memory.events[-1]) if getattr(memory, "events", None) else None
    roof_context = memory.record_roof_plane(stage, estimate, observer_world=observer_world)
    current_event = dict(memory.events[-1]) if getattr(memory, "events", None) else None
    rejection_reason = ""
    if roof_context is None:
        if current_event != previous_event and str((current_event or {}).get("type", "")).startswith("reject_roof_plane"):
            rejection_reason = str(current_event.get("type"))
        elif float(estimate.confidence) < float(memory.config.get("ABOVE_ROOF_MIN_CONFIDENCE", 0.45)):
            rejection_reason = "roof_plane_confidence_below_threshold"
        else:
            rejection_reason = "roof_plane_memory_association_rejected"
    print(
        f"  [RoofPlane] source={source} accepted={roof_context is not None} "
        f"reason={rejection_reason or estimate.reason} "
        f"z={float(estimate.roof_z_world):.2f}m depth={float(estimate.center_depth_m):.2f}m "
        f"confidence={estimate.confidence:.2f} valid={estimate.valid_ratio:.2f} "
        f"coverage={estimate.coverage_ratio:.2f} center={estimate.center_support_ratio:.2f} "
        f"z_mad={float(estimate.z_mad_m or 0.0):.3f}m "
        f"normal={float(estimate.normal_error_deg or 0.0):.2f}deg "
        f"full_frame={estimate.full_frame}"
    )
    if roof_context is not None and state is not None:
        state.update(memory_summary=memory.summary(stage))
    if roof_context is not None and bool(roof_context.get("trusted", False)):
        above_state = _above_stage_state(objects, stage, observer_world)
        above_state.roof_acquire_origin_world = None
        above_state.roof_acquire_direction_world = None
        above_state.roof_acquire_goal_distance_m = 0.0
        above_state.require_roof_before_next_completion = False
        above_state.completion_resume_pose = None
    return estimate
