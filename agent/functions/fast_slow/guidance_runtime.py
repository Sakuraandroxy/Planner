"""Geometry-only guidance guards for the fast-slow planner.

The Qwen planner is still responsible for choosing a useful local trajectory,
but a locked target is a hard geometric contract.  This module keeps that
contract small and testable: accepted path endpoints must point at the active
target and must make progress toward it.  When they do not, a short body-frame
target-directed leg is generated so the slow planner cannot repeatedly send a
long, almost-forward path past the target.
"""

from __future__ import annotations

import math
from typing import Any

from agent.functions.memory.geometry import world_to_body


def _signed_angle_deg(target: float, reference: float) -> float:
    return (float(target) - float(reference) + 180.0) % 360.0 - 180.0


def _xy_distance(point: list[float] | tuple[float, ...]) -> float:
    return math.hypot(float(point[0]), float(point[1])) if len(point) >= 2 else 0.0


def _path_length(points: list) -> float:
    previous = [0.0, 0.0, 0.0]
    total = 0.0
    for point in list(points or []):
        if point is None or len(point) < 3:
            continue
        current = [float(value) for value in point[:3]]
        total += math.sqrt(sum((current[i] - previous[i]) ** 2 for i in range(3)))
        previous = current
    return total


def _limit_path(points: list, max_length: float) -> tuple[list, bool]:
    if not points or max_length <= 0.0:
        return points, False
    previous = [0.0, 0.0, 0.0]
    traveled = 0.0
    limited: list[list[float]] = []
    for point in points:
        current = [float(value) for value in point[:3]]
        segment = [current[i] - previous[i] for i in range(3)]
        length = math.sqrt(sum(value * value for value in segment))
        if traveled + length <= max_length + 1e-6:
            limited.append([round(value, 3) for value in current])
            traveled += length
            previous = current
            continue
        remaining = max(0.0, max_length - traveled)
        if remaining > 1e-3 and length > 1e-6:
            scale = remaining / length
            limited.append([
                round(previous[i] + segment[i] * scale, 3)
                for i in range(3)
            ])
        return limited, True
    return limited, False


def _straight_body_path(direction_xy: list[float], distance_m: float, *, spacing_m: float = 2.5) -> list[list[float]]:
    distance = max(0.0, float(distance_m))
    norm = math.hypot(float(direction_xy[0]), float(direction_xy[1]))
    if distance <= 1e-6 or norm <= 1e-6:
        return []
    unit = [float(direction_xy[0]) / norm, float(direction_xy[1]) / norm]
    count = max(1, min(8, int(math.ceil(distance / max(0.5, float(spacing_m))))))
    return [
        [
            round(unit[0] * distance * index / count, 3),
            round(unit[1] * distance * index / count, 3),
            0.0,
        ]
        for index in range(1, count + 1)
    ]


def _bearing_direct_path(angle_deg: float, distance_m: float, *, spacing_m: float = 2.5) -> list[list[float]]:
    angle = math.radians(float(angle_deg))
    return _straight_body_path(
        [math.cos(angle), math.sin(angle)],
        distance_m,
        spacing_m=spacing_m,
    )


def _extend_target_horizon(
    points: list,
    target_xy: list[float],
    target_distance: float,
    completion_distance: float,
    horizon_m: float,
    *,
    spacing_m: float = 2.5,
) -> list[list[float]]:
    """Continue an already-convergent path until the rolling horizon is met."""
    if not points or horizon_m <= 0.0:
        return points
    previous = [float(value) for value in points[-1][:3]]
    path_length = _path_length(points)
    if path_length >= horizon_m - 0.25:
        return points
    target_norm = math.hypot(float(target_xy[0]), float(target_xy[1]))
    if target_norm <= 1e-6:
        return points
    target_unit = [float(target_xy[0]) / target_norm, float(target_xy[1]) / target_norm]
    projected = previous[0] * target_unit[0] + previous[1] * target_unit[1]
    available = max(0.0, float(target_distance) - float(completion_distance) - projected)
    extension = min(float(horizon_m) - path_length, available)
    if extension <= 0.75:
        return points
    count = max(1, int(math.ceil(extension / max(0.5, float(spacing_m)))))
    out = [list(point[:3]) for point in points]
    for index in range(1, count + 1):
        distance = extension * index / count
        out.append([
            round(previous[0] + target_unit[0] * distance, 3),
            round(previous[1] + target_unit[1] * distance, 3),
            round(previous[2], 3),
        ])
    return out


def _bearing_observation(objects: Any, stage: Any):
    tracker = getattr(objects, "target_bearing_tracker", None)
    if tracker is None:
        return None, False
    stage_key = None
    try:
        from agent.functions.fast_slow.completion_pipeline import CompletionPipeline

        stage_key = CompletionPipeline.stage_key(stage)
    except Exception:
        stage_key = (getattr(stage, "index", 0), str(getattr(stage, "instruction", "")))
    activation = tracker.activation_guard(stage_key) if hasattr(tracker, "activation_guard") else None
    observation = activation or (tracker.current(stage_key) if hasattr(tracker, "current") else None)
    return observation, activation is not None


def apply_bearing_path_guard(objects: Any, stage: Any, cumulative_waypoints: list, *, selection_pos, selection_yaw):
    """Constrain RGB-only guidance to a fresh target bearing.

    A bearing is intentionally short-lived and does not create a 3-D target.
    It is nevertheless enough to reject a long straight path that would miss
    the target by many degrees while depth is still unavailable.
    """
    observation, is_prebind = _bearing_observation(objects, stage)
    if observation is None:
        return cumulative_waypoints, ""
    # A metric lock is handled by the formal target guard.  An activation
    # bearing remains valid until its first non-empty path is queued.
    if not is_prebind and str(getattr(observation, "depth_state", "")) == "metric":
        return cumulative_waypoints, ""
    memory = getattr(objects, "mission_memory", None)
    config = getattr(memory, "config", {}) or {}
    if is_prebind:
        max_leg = float(config.get("PREBIND_MAX_LEG_M", 6.0))
        max_deviation = float(config.get("PREBIND_MAX_PATH_DEVIATION_DEG", 5.0))
        prefix = "prebind_"
    else:
        max_leg = float(config.get("BEARING_MAX_LEG_M", 8.0))
        max_deviation = float(config.get("BEARING_MAX_PATH_DEVIATION_DEG", 5.0))
        prefix = "bearing_"
    if max_leg <= 0.0:
        return cumulative_waypoints, ""
    limited, was_limited = _limit_path(cumulative_waypoints, max_leg)
    target_angle = float(observation.relative_to_yaw(float(selection_yaw)))
    endpoint = list(limited[-1]) if limited else [0.0, 0.0, 0.0]
    endpoint_angle = math.degrees(math.atan2(float(endpoint[1]), float(endpoint[0]))) if _xy_distance(endpoint) > 1e-6 else 0.0
    deviation = abs(_signed_angle_deg(endpoint_angle, target_angle))
    target_range = getattr(observation, "range_hint_m", None)
    target_range = float(target_range) if target_range is not None and float(target_range) > 0.0 else max_leg
    target_range = min(max_leg, target_range)
    # Bearing-only legs are deliberately a single short command.  Depth will
    # be reacquired after it, and splitting this first leg into tiny points
    # makes the old sliding-window queue appear to pause at every point.
    desired = _bearing_direct_path(target_angle, target_range, spacing_m=max_leg + 1.0)
    endpoint_progress = float(endpoint[0]) * math.cos(math.radians(target_angle)) + float(endpoint[1]) * math.sin(math.radians(target_angle))
    if (
        limited
        and endpoint[0] > 0.0
        and deviation <= max_deviation
        and endpoint_progress >= max(0.5, target_range * 0.70)
    ):
        return limited, f"{prefix}limit_{max_leg:.1f}m" if was_limited else ""
    # Keep the historical reason prefix for log/test compatibility while the
    # configured 5-degree corridor supplies the new strict behavior.
    reason_prefix = "prebind_" if is_prebind else ""
    return desired, f"{reason_prefix}replace_bearing_deviation_{deviation:.1f}deg"


def apply_locked_target_direction_guard(
    objects: Any,
    stage: Any,
    cumulative_waypoints: list,
    *,
    selection_pos,
    selection_yaw,
):
    """Force every formal locked-target leg to converge toward its target.

    The returned waypoints are incremental body-frame cumulative points.  The
    guard only changes a path when its endpoint is outside the configured
    angular corridor or fails to reduce target distance; otherwise Qwen's
    obstacle-aware shape is preserved.
    """
    if not cumulative_waypoints:
        return cumulative_waypoints, ""
    memory = getattr(objects, "mission_memory", None)
    if memory is None or not callable(getattr(memory, "primary_instance", None)):
        return cumulative_waypoints, ""
    instance = memory.primary_instance(stage)
    if instance is None:
        return cumulative_waypoints, ""
    target_world = list(getattr(instance, "target_world", None) or [])
    current = list(selection_pos or [])
    if len(target_world) < 3 or len(current) < 3:
        return cumulative_waypoints, ""
    body_target = world_to_body(target_world, current, float(selection_yaw))
    relation = str(getattr(stage, "relation", "") or "").lower()
    horizontal_target = _xy_distance(body_target)
    if relation in {"above", "over", "on_top_of"}:
        target_distance = horizontal_target
        if target_distance <= 1e-6:
            return cumulative_waypoints, ""
        completion_radius = max(
            0.8,
            float(getattr(instance, "footprint_radius_m", 1.5) or 1.5)
            + float(config_value(memory, "ABOVE_HORIZONTAL_RADIUS_M", 3.5)),
        )
        desired_distance = max(0.0, target_distance - completion_radius)
        target_xy = [body_target[0], body_target[1]]
    else:
        target_distance = horizontal_target
        if target_distance <= 1e-6:
            return cumulative_waypoints, ""
        approach_radius = max(
            0.8,
            float(config_value(memory, "NEAR_APPROACH_RADIUS_M", 4.5)),
        )
        desired_distance = max(0.0, target_distance - approach_radius)
        target_xy = [body_target[0], body_target[1]]
    if desired_distance <= 0.25:
        return cumulative_waypoints, ""
    config = getattr(memory, "config", {}) or {}
    max_leg = max(1.0, float(config.get("TARGET_GUIDANCE_LEG_M", config.get("PATH_MAX_GUIDED_LEG_M", 6.0))))
    # Once a metric target is locked, a fixed 6 m corrective leg is shorter
    # than Qwen's 5-6 s response time.  Preserve the target-direction safety
    # contract while extending the fallback to the same rolling horizon used
    # by the queue controller.  RGB-only prebinding remains governed by its
    # separate short bearing leg.
    controller = getattr(objects, "controller", None)
    horizon_fn = getattr(controller, "planning_horizon_m", None)
    if callable(horizon_fn):
        try:
            nominal_speed = max(0.1, float(config.get("AIRSIM_VELOCITY_MPS", 2.0)))
            max_leg = max(max_leg, float(horizon_fn(nominal_speed)))
        except (TypeError, ValueError):
            pass
    max_deviation = max(0.5, float(config.get("TARGET_PATH_DIRECTION_ERROR_DEG", 5.0)))
    direct_distance = min(max_leg, desired_distance)
    target_angle = math.degrees(math.atan2(float(target_xy[1]), float(target_xy[0])))
    endpoint = list(cumulative_waypoints[-1])
    endpoint_distance = _xy_distance(endpoint)
    endpoint_angle = math.degrees(math.atan2(float(endpoint[1]), float(endpoint[0]))) if endpoint_distance > 1e-6 else 0.0
    deviation = abs(_signed_angle_deg(endpoint_angle, target_angle))
    target_unit = [float(target_xy[0]) / horizontal_target, float(target_xy[1]) / horizontal_target]
    endpoint_projection = float(endpoint[0]) * target_unit[0] + float(endpoint[1]) * target_unit[1]
    current_distance = target_distance
    endpoint_remaining = math.hypot(float(body_target[0]) - float(endpoint[0]), float(body_target[1]) - float(endpoint[1]))
    min_progress = max(0.1, float(config.get("TARGET_GUIDANCE_MIN_PROGRESS_M", 0.15)))
    behind_threshold = max(1.0, float(config.get("PATH_TARGET_BEHIND_X_M", 2.0)))
    if float(body_target[0]) < -behind_threshold and abs(target_angle) > 90.0:
        # Let the bounded reorientation policy rotate once before issuing a
        # backwards path.  Keeping the old path here would re-create the
        # overshoot loop, so return a short lateral correction only if one is
        # already underway; an empty queue will be handled by reorientation.
        return cumulative_waypoints, "target_behind_reorient_required"
    accepts = bool(
        endpoint_distance > 1e-6
        and deviation <= max_deviation
        and endpoint_projection >= min_progress
        and endpoint_remaining <= current_distance - min_progress
    )
    if accepts:
        # Preserve Qwen's shape, but append a target-directed continuation when
        # the accepted leg is shorter than the slow-planner horizon.  This is
        # what keeps a valid 4-6 m Qwen answer from exhausting the queue while
        # the next 5-6 s request is still running.
        extended = _extend_target_horizon(
            cumulative_waypoints,
            target_xy,
            target_distance,
            completion_radius if relation in {"above", "over", "on_top_of"} else approach_radius,
            max_leg,
        )
        if len(extended) > len(cumulative_waypoints):
            return extended, f"extend_target_horizon_{_path_length(extended):.1f}m"
        return cumulative_waypoints, ""
    direct = _straight_body_path(target_xy, direct_distance, spacing_m=2.5)
    if not direct:
        return cumulative_waypoints, ""
    reason = f"replace_target_direction_{deviation:.1f}deg"
    if endpoint_remaining > current_distance - min_progress:
        reason += "_no_progress"
    return direct, reason


def config_value(memory: Any, key: str, default: float) -> float:
    try:
        return float((getattr(memory, "config", {}) or {}).get(key, default))
    except (TypeError, ValueError):
        return float(default)


__all__ = [
    "apply_bearing_path_guard",
    "apply_locked_target_direction_guard",
]
