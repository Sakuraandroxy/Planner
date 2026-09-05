"""Planning support helpers for the fast-slow AirSim runtime.

This module owns the low-level planning context, path clipping, and geometry
guards used by the runtime.  The main planning submission and execution loop
remain in ``runtime.py``; the exported helpers are intentionally side-effect
compatible with their former locations.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

from config import cfg

from agent.functions.common.config_access import function_section
from agent.functions.memory.geometry import world_to_body

if TYPE_CHECKING:
    from agent.functions.fast_slow.runtime_context import RuntimeObjects
else:
    RuntimeObjects = Any


def _planning_geometry_context(objects, client, stage, pos_now, yaw_now, images):
    """Build a compact world/navigation-frame description for the VLM.

    Camera roles come from the pose carried by each Camera API response.  The
    image list order is only a transport order and does not imply front/down.
    """
    camera_images = []
    camera_views = []
    seen_images = set()
    seen_cameras = set()
    frame_lookup = getattr(client, "camera_frame_for_image", None)
    for image in list(images or []):
        if image is None or id(image) in seen_images:
            continue
        seen_images.add(id(image))
        camera_frame = (
            frame_lookup(image)
            if callable(frame_lookup)
            else getattr(image, "camera_frame", None)
        )
        camera_images.append(image)
        if camera_frame is None or camera_frame.camera_id in seen_cameras:
            continue
        seen_cameras.add(camera_frame.camera_id)
        rgb_intrinsics = getattr(camera_frame, "rgb_intrinsics", None)
        depth_intrinsics = getattr(camera_frame, "depth_intrinsics", None)
        camera_views.append({
            "camera_id": str(camera_frame.camera_id),
            "capture_id": str(camera_frame.capture_id),
            "camera_position_world": [round(float(v), 3) for v in camera_frame.camera_position_world[:3]],
            "optical_axis_world": [round(float(v), 6) for v in camera_frame.optical_axis_world[:3]],
            "rgb_size": (
                [int(rgb_intrinsics.width), int(rgb_intrinsics.height)]
                if rgb_intrinsics is not None
                else list(getattr(image, "size", []) or [])[:2]
            ),
            "rgb_hfov_deg": (
                round(float(rgb_intrinsics.horizontal_fov_deg), 3)
                if rgb_intrinsics is not None
                else None
            ),
            "depth_size": (
                [int(depth_intrinsics.width), int(depth_intrinsics.height)]
                if depth_intrinsics is not None
                else None
            ),
            "depth_available": bool(getattr(camera_frame, "depth", None) is not None),
        })

    memory = getattr(objects, "mission_memory", None)
    if memory is not None and hasattr(memory, "navigation_context"):
        context = memory.navigation_context(stage, pos_now, yaw_now)
    else:
        context = {
            "coordinate_convention": "AirSim NED; +x forward, +y right, +z down in navigation frame",
            "current_origin_world": [float(v) for v in pos_now[:3]],
            "current_yaw_deg": float(yaw_now),
        }
    context["camera_views"] = camera_views
    context["image_camera_order"] = [
        str(getattr(getattr(image, "camera_frame", None), "camera_id", f"view_{index}"))
        for index, image in enumerate(camera_images)
    ]

    avoider = getattr(objects, "obstacle_avoider", None)
    obstacle_summary = avoider.summary() if avoider is not None and hasattr(avoider, "summary") else {}
    obstacle_samples = []
    for sample in list(obstacle_summary.get("sample") or []):
        world = list(sample.get("world") or [])
        if len(world) < 3:
            continue
        obstacle_samples.append({
            "nav_xyz": [round(float(v), 2) for v in world_to_body(world, pos_now, yaw_now)],
            "confidence": float(sample.get("confidence", 0.0) or 0.0),
            "age_s": float(sample.get("age_s", 0.0) or 0.0),
        })
    context["obstacles"] = {
        "cell_count": int(obstacle_summary.get("cells", 0) or 0),
        "samples_nav": obstacle_samples,
    }
    return camera_images, context


def _format_waypoints(waypoints, limit: int = 5) -> str:
    points = []
    for wp in list(waypoints or [])[:limit]:
        if not isinstance(wp, (list, tuple)) or len(wp) < 3:
            continue
        points.append(f"[{float(wp[0]):.2f}, {float(wp[1]):.2f}, {float(wp[2]):.2f}]")
    suffix = " ..." if len(list(waypoints or [])) > limit else ""
    return "[" + ", ".join(points) + "]" + suffix if points else "[]"


def _candidate_trace_text(cand) -> str:
    if cand is None:
        return "none"
    if isinstance(cand, dict):
        getter = cand.get
    else:
        getter = lambda key, default=None: getattr(cand, key, default)
    breakdown = getter("score_breakdown", {}) or {}
    parts = [
        f"score={float(getter('pre_score', 0.0) or 0.0):.4f}",
        f"conf={float(getter('confidence', 0.0) or 0.0):.2f}",
        f"source={getter('source', '')}",
        f"wp={_format_waypoints(getter('waypoints', []))}",
    ]
    if breakdown:
        parts.append("breakdown=" + ",".join(f"{k}:{float(v):.3f}" for k, v in breakdown.items()))
    return " ".join(parts)


def _memory_path_clip_radius(objects: RuntimeObjects) -> float:
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    distance_cfg = function_section(cfg, "DISTANCE_ESTIMATION")
    return float(memory_cfg.get(
        "PATH_CLIP_RADIUS_M",
        distance_cfg.get(
            "TRIGGER_RADIUS_M",
            getattr(objects.completion_checker, "stop_depth", cfg.get("AGENT", {}).get("STOP_DEPTH_THRESHOLD", 4.0)),
        ),
    ))


def _memory_near_standoff_radius(objects: RuntimeObjects, memory_context: dict, base_radius: float) -> float:
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    target = memory_context.get("target_body") or []
    radius = max(float(base_radius), float(memory_cfg.get("NEAR_STANDOFF_M", base_radius)))
    footprint = float(memory_context.get("footprint_radius_m", 1.5) or 1.5)
    uncertainty = float(memory_context.get("uncertainty_m", 0.0) or 0.0)
    radius = max(
        radius,
        footprint
        + float(memory_cfg.get("NEAR_TARGET_EXTRA_STANDOFF_M", 0.0))
        + min(max(uncertainty, 0.0), float(memory_cfg.get("STANDOFF_UNCERTAINTY_CAP_M", 0.0))) * 0.35,
    )
    # 低空贴近车辆/箱子时不要按目标中心飞，额外留一点外圈距离，避免擦到碰撞盒。
    if len(target) >= 3 and abs(float(target[2])) <= float(memory_cfg.get("LOW_ALTITUDE_Z_DELTA_M", 3.0)):
        radius += float(memory_cfg.get("LOW_ALTITUDE_EXTRA_STANDOFF_M", 0.0))
    return max(0.8, radius)


def _memory_near_approach_radius_from_values(
    memory_cfg: dict,
    *,
    footprint: float,
    uncertainty: float,
    outer_radius: float,
) -> float:
    explicit = memory_cfg.get("NEAR_APPROACH_RADIUS_M", None)
    if explicit is None:
        return max(0.8, float(outer_radius))
    radius = float(explicit)
    radius = max(
        radius,
        float(footprint)
        + float(memory_cfg.get("NEAR_APPROACH_TARGET_CLEARANCE_M", 2.2))
        + min(max(float(uncertainty), 0.0), float(memory_cfg.get("NEAR_APPROACH_UNCERTAINTY_CAP_M", 1.5))) * 0.25,
    )
    min_gap = float(memory_cfg.get("NEAR_APPROACH_OUTER_GAP_M", 1.0))
    if float(outer_radius) > min_gap + 0.8:
        radius = min(radius, float(outer_radius) - min_gap)
    return max(0.8, radius)


def _memory_near_approach_radius(objects: RuntimeObjects, memory_context: dict, outer_radius: float) -> float:
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    return _memory_near_approach_radius_from_values(
        memory_cfg,
        footprint=float(memory_context.get("footprint_radius_m", 1.5) or 1.5),
        uncertainty=float(memory_context.get("uncertainty_m", 0.0) or 0.0),
        outer_radius=float(outer_radius),
    )


def _apply_memory_path_guard(objects: RuntimeObjects, stage, cumulative_waypoints: list, memory_context: dict) -> tuple[list, str]:
    """Clip or replace a Qwen path so it cannot fly far past the locked memory target."""
    memory_cfg = getattr(getattr(objects, "mission_memory", None), "config", {}) or {}
    if not bool(memory_cfg.get("PATH_CLIP_ENABLED", True)):
        return cumulative_waypoints, ""
    if not memory_context or not bool(memory_context.get("enabled", False)):
        return cumulative_waypoints, ""
    target = memory_context.get("target_body") or []
    if len(target) < 3 or not cumulative_waypoints:
        return cumulative_waypoints, ""
    target = [float(target[0]), float(target[1]), float(target[2])]
    target_dist = _norm3(target)
    if target_dist <= 1e-6:
        return [], "already_at_memory_target"

    relation = str(memory_context.get("relation", "near") or "near").lower()
    base_radius = _memory_path_clip_radius(objects)
    if relation == "above":
        outer_radius = max(
            base_radius,
            float(memory_context.get("footprint_radius_m", 1.5) or 1.5)
            + float(memory_cfg.get("ABOVE_HORIZONTAL_RADIUS_M", 3.5)),
        )
        # 已经进入“上方”的水平范围时，不要把所有轨迹清空；还需要允许Qwen继续做高度/位置微调。
        if _guard_distance([0.0, 0.0, 0.0], target, relation) <= outer_radius:
            return cumulative_waypoints, ""
        radius = float(memory_cfg.get("ABOVE_APPROACH_RADIUS_M", max(2.0, min(outer_radius * 0.65, outer_radius - 1.0))))
    else:
        outer_radius = _memory_near_standoff_radius(objects, memory_context, base_radius)
        radius = _memory_near_approach_radius(objects, memory_context, outer_radius)
    radius = max(0.8, float(radius))
    current_dist = _guard_distance([0.0, 0.0, 0.0], target, relation)
    start_inside_outer = current_dist <= float(outer_radius)

    clipped = _clip_path_at_target_radius(
        cumulative_waypoints,
        target,
        radius,
        relation=relation,
        near_max_descent_m=float(memory_cfg.get("PATH_NEAR_MAX_DESCENT_M", 0.2)),
        near_max_climb_m=float(memory_cfg.get("PATH_NEAR_MAX_CLIMB_M", 0.5)),
        empty_if_start_inside=not start_inside_outer,
    )
    if clipped is not None:
        clipped, leg_limited = _limit_cumulative_path_length(clipped, memory_cfg)
        leg_reason = f"_leg_{float(memory_cfg.get('PATH_MAX_GUIDED_LEG_M', 0.0)):.1f}m" if leg_limited else ""
        return clipped, f"clip_enter_approach_{radius:.1f}m{leg_reason}"

    endpoint = [float(v) for v in cumulative_waypoints[-1][:3]]
    endpoint_dist = _distance3_body(endpoint, target)
    closest_dist = _closest_path_distance_to_target(cumulative_waypoints, target, relation=relation)
    target_xy_norm = math.sqrt(target[0] * target[0] + target[1] * target[1])
    endpoint_projection = _project_xy(endpoint, target)
    overshoot = target_xy_norm > 1e-6 and endpoint_projection > target_xy_norm + radius
    if start_inside_outer:
        # 已在完成圆内部时，只拦截明显飞离目标的路径，允许继续向圆内部微调。
        diverging = endpoint_dist > max(
            float(outer_radius) * float(memory_cfg.get("PATH_EXIT_RADIUS_RATIO", 1.20)),
            current_dist + float(memory_cfg.get("PATH_EXIT_MARGIN_M", 2.0)),
        )
    else:
        diverging = endpoint_dist > target_dist * float(memory_cfg.get("PATH_DIVERGE_RATIO", 0.90))
    missed_close = closest_dist <= radius * float(memory_cfg.get("PATH_CLOSE_MISS_RATIO", 1.35))
    target_behind = (
        not start_inside_outer
        and target[0] < -float(memory_cfg.get("PATH_TARGET_BEHIND_X_M", 2.0))
    )
    if overshoot or diverging or missed_close or target_behind:
        direct = _direct_memory_waypoint(
            target,
            radius,
            relation=relation,
            near_max_descent_m=float(memory_cfg.get("PATH_NEAR_MAX_DESCENT_M", 0.2)),
            near_max_climb_m=float(memory_cfg.get("PATH_NEAR_MAX_CLIMB_M", 0.5)),
        )
        reason_bits = []
        if overshoot:
            reason_bits.append("overshoot")
        if diverging:
            reason_bits.append("diverging")
        if missed_close:
            reason_bits.append("near_miss")
        if target_behind:
            reason_bits.append("target_behind")
        direct, leg_limited = _limit_cumulative_path_length(direct, memory_cfg)
        if leg_limited:
            reason_bits.append(f"leg_{float(memory_cfg.get('PATH_MAX_GUIDED_LEG_M', 0.0)):.1f}m")
        return direct, "replace_" + "_".join(reason_bits)
    limited, leg_limited = _limit_cumulative_path_length(cumulative_waypoints, memory_cfg)
    if leg_limited:
        return limited, f"clip_active_leg_{float(memory_cfg.get('PATH_MAX_GUIDED_LEG_M', 0.0)):.1f}m"
    return cumulative_waypoints, ""


def _limit_cumulative_path_length(cumulative_waypoints: list, memory_cfg: dict) -> tuple[list, bool]:
    """Bound one newly appended memory-guided leg while preserving its shape."""
    max_length = float(memory_cfg.get("PATH_MAX_GUIDED_LEG_M", 0.0) or 0.0)
    if max_length <= 0.0 or not cumulative_waypoints:
        return cumulative_waypoints, False
    prev = [0.0, 0.0, 0.0]
    traveled = 0.0
    limited = []
    for waypoint in cumulative_waypoints:
        cur = [float(v) for v in waypoint[:3]]
        segment = [cur[i] - prev[i] for i in range(3)]
        segment_length = _norm3(segment)
        if traveled + segment_length <= max_length + 1e-6:
            limited.append([round(v, 3) for v in cur])
            traveled += segment_length
            prev = cur
            continue
        remaining = max(0.0, max_length - traveled)
        if remaining > 1e-3 and segment_length > 1e-6:
            scale = remaining / segment_length
            limited.append([
                round(prev[i] + segment[i] * scale, 3)
                for i in range(3)
            ])
        return limited, True
    return limited, False


def _clip_path_at_target_radius(
    cumulative_waypoints: list,
    target: list[float],
    radius: float,
    *,
    relation: str = "near",
    near_max_descent_m: float = 0.2,
    near_max_climb_m: float = 0.5,
    empty_if_start_inside: bool = True,
):
    prev = [0.0, 0.0, 0.0]
    prev_dist = _guard_distance(prev, target, relation)
    if prev_dist <= radius:
        return [] if empty_if_start_inside else None
    out = []
    for waypoint in cumulative_waypoints:
        cur = [float(v) for v in waypoint[:3]]
        cur_dist = _guard_distance(cur, target, relation)
        if cur_dist <= radius:
            if relation == "above":
                hit = _segment_circle_entry_xy(prev, cur, target, radius) or cur
            else:
                hit = _segment_circle_entry_xy(prev, cur, target, radius) or cur
                # “旁边/附近”不应该把高度也插值到目标中心；低空靠近车辆时尤其容易撞。
                hit[2] = _clamp(hit[2], -abs(float(near_max_climb_m)), abs(float(near_max_descent_m)))
            out.append([round(hit[0], 3), round(hit[1], 3), round(hit[2], 3)])
            return out
        out.append([round(cur[0], 3), round(cur[1], 3), round(cur[2], 3)])
        prev = cur
    return None


def _segment_sphere_entry(a: list[float], b: list[float], center: list[float], radius: float):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    ax = a[0] - center[0]
    ay = a[1] - center[1]
    az = a[2] - center[2]
    qa = dx * dx + dy * dy + dz * dz
    if qa <= 1e-9:
        return None
    qb = 2.0 * (ax * dx + ay * dy + az * dz)
    qc = ax * ax + ay * ay + az * az - radius * radius
    disc = qb * qb - 4.0 * qa * qc
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    candidates = [(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)]
    valid = [t for t in candidates if 0.0 <= t <= 1.0]
    if not valid:
        return None
    t = min(valid)
    return [a[0] + t * dx, a[1] + t * dy, a[2] + t * dz]


def _segment_circle_entry_xy(a: list[float], b: list[float], center: list[float], radius: float):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    ax = a[0] - center[0]
    ay = a[1] - center[1]
    qa = dx * dx + dy * dy
    if qa <= 1e-9:
        return None
    qb = 2.0 * (ax * dx + ay * dy)
    qc = ax * ax + ay * ay - radius * radius
    disc = qb * qb - 4.0 * qa * qc
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    candidates = [(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)]
    valid = [t for t in candidates if 0.0 <= t <= 1.0]
    if not valid:
        return None
    t = min(valid)
    return [a[0] + t * dx, a[1] + t * dy, a[2] + t * (b[2] - a[2])]


def _direct_memory_waypoint(
    target: list[float],
    radius: float,
    *,
    relation: str = "near",
    near_max_descent_m: float = 0.2,
    near_max_climb_m: float = 0.5,
) -> list:
    dist = _norm3(target) if relation == "above" else math.sqrt(target[0] * target[0] + target[1] * target[1])
    if dist <= max(radius, 1e-6):
        return []
    scale = max(0.0, (dist - radius) / dist)
    z = target[2] * scale if relation == "above" else _clamp(0.0, -abs(float(near_max_climb_m)), abs(float(near_max_descent_m)))
    return [[
        round(target[0] * scale, 3),
        round(target[1] * scale, 3),
        round(z, 3),
    ]]


def _closest_path_distance_to_target(cumulative_waypoints: list, target: list[float], *, relation: str = "near") -> float:
    prev = [0.0, 0.0, 0.0]
    best = _guard_distance(prev, target, relation)
    for waypoint in cumulative_waypoints:
        cur = [float(v) for v in waypoint[:3]]
        if relation == "above":
            best = min(best, _segment_point_distance_xy(prev, cur, target))
        else:
            best = min(best, _segment_point_distance_xy(prev, cur, target))
        prev = cur
    return best


def _segment_point_distance(a: list[float], b: list[float], point: list[float]) -> float:
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    vz = b[2] - a[2]
    wx = point[0] - a[0]
    wy = point[1] - a[1]
    wz = point[2] - a[2]
    denom = vx * vx + vy * vy + vz * vz
    if denom <= 1e-9:
        return _distance3_body(a, point)
    t = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / denom))
    closest = [a[0] + t * vx, a[1] + t * vy, a[2] + t * vz]
    return _distance3_body(closest, point)


def _segment_point_distance_xy(a: list[float], b: list[float], point: list[float]) -> float:
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    wx = point[0] - a[0]
    wy = point[1] - a[1]
    denom = vx * vx + vy * vy
    if denom <= 1e-9:
        return math.sqrt((a[0] - point[0]) ** 2 + (a[1] - point[1]) ** 2)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    closest_x = a[0] + t * vx
    closest_y = a[1] + t * vy
    return math.sqrt((closest_x - point[0]) ** 2 + (closest_y - point[1]) ** 2)


def _guard_distance(point: list[float], target: list[float], relation: str = "near") -> float:
    if str(relation or "near").lower() == "above":
        return math.sqrt((float(point[0]) - float(target[0])) ** 2 + (float(point[1]) - float(target[1])) ** 2)
    return math.sqrt((float(point[0]) - float(target[0])) ** 2 + (float(point[1]) - float(target[1])) ** 2)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def _project_xy(point: list[float], direction: list[float]) -> float:
    denom = math.sqrt(direction[0] * direction[0] + direction[1] * direction[1])
    if denom <= 1e-9:
        return 0.0
    return (point[0] * direction[0] + point[1] * direction[1]) / denom


def _norm3(point: list[float]) -> float:
    return math.sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2])


def _distance3_body(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _apply_obstacle_path_guard(
    objects: RuntimeObjects,
    cumulative_waypoints: list,
    *,
    selection_pos,
    selection_yaw: float,
    memory_context: dict,
) -> tuple[list, str, Any]:
    avoider = getattr(objects, "obstacle_avoider", None)
    if avoider is None or not getattr(avoider, "enabled", False):
        return cumulative_waypoints, "", None
    result = avoider.filter_cumulative_waypoints(
        cumulative_waypoints,
        current_world=selection_pos,
        yaw_deg=float(selection_yaw),
        memory_context=memory_context,
    )
    if not getattr(result, "changed", False):
        return cumulative_waypoints, "", result
    obstacle = getattr(result, "obstacle_body", None)
    obstacle_text = "" if obstacle is None else f" obstacle_body={obstacle}"
    return (
        list(getattr(result, "waypoints", []) or []),
        f"{result.reason}{obstacle_text}",
        result,
    )


__all__ = [
    "_planning_geometry_context",
    "_format_waypoints",
    "_candidate_trace_text",
    "_memory_path_clip_radius",
    "_memory_near_standoff_radius",
    "_memory_near_approach_radius_from_values",
    "_memory_near_approach_radius",
    "_apply_memory_path_guard",
    "_limit_cumulative_path_length",
    "_clip_path_at_target_radius",
    "_segment_sphere_entry",
    "_segment_circle_entry_xy",
    "_direct_memory_waypoint",
    "_closest_path_distance_to_target",
    "_segment_point_distance",
    "_segment_point_distance_xy",
    "_guard_distance",
    "_clamp",
    "_project_xy",
    "_norm3",
    "_distance3_body",
    "_apply_obstacle_path_guard",
]
