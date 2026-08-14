"""Geometry helpers for lightweight target memory."""

from __future__ import annotations

import math
from typing import Any, Iterable, Optional, Sequence


def point3(value: Sequence[float]) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def distance3(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def horizontal_distance(a: Sequence[float], b: Sequence[float]) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return math.sqrt(dx * dx + dy * dy)


def bbox_area_ratio(detection: Any, image: Any) -> float:
    if detection is None or not getattr(detection, "bbox", None) or image is None or not hasattr(image, "size"):
        return 0.0
    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return 0.0
    x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
    box_w = max(0.0, min(width, x2) - max(0.0, x1))
    box_h = max(0.0, min(height, y2) - max(0.0, y1))
    return (box_w * box_h) / max(width * height, 1.0)


def bbox_quality(detection: Any, image: Any) -> float:
    """Map detector score and bbox shape into a conservative 0..1 quality."""
    if detection is None or not getattr(detection, "visible", False):
        return 0.0
    score = max(0.0, min(1.0, float(getattr(detection, "score", 0.0) or 0.0)))
    if image is None or not getattr(detection, "bbox", None) or not hasattr(image, "size"):
        return score
    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return score
    x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
    box_w = max(0.0, min(width, x2) - max(0.0, x1))
    box_h = max(0.0, min(height, y2) - max(0.0, y1))
    if box_w <= 1.0 or box_h <= 1.0:
        return 0.0
    span_x = box_w / width
    span_y = box_h / height
    area = span_x * span_y
    if span_x >= 0.90 or span_y >= 0.90:
        return 0.0

    quality = 1.0
    # 太小的框通常对应远距离/噪声，不确定性会明显增大。
    if area <= 0.0002:
        quality *= 0.35
    elif area <= 0.001:
        quality *= 0.65
    elif area >= 0.55:
        quality *= 0.08
    elif area >= 0.30 or span_x >= 0.75 or span_y >= 0.75:
        quality *= 0.25
    elif area >= 0.18:
        quality *= 0.50
    if x1 <= 2.0 or y1 <= 2.0 or x2 >= width - 2.0 or y2 >= height - 2.0:
        quality *= 0.55
    return max(0.0, min(1.0, score * quality))


def estimate_detection_world(
    detection: Any,
    image: Any,
    observer_world: Sequence[float],
    observer_yaw_deg: float,
    *,
    memory_config: Optional[dict] = None,
    sim_config: Optional[dict] = None,
) -> Optional[list[float]]:
    """Project a depth-backed bbox center into AirSim world coordinates."""
    memory_config = memory_config or {}
    sim_config = sim_config or {}
    if detection is None or not getattr(detection, "visible", False):
        return None
    bbox = list(getattr(detection, "bbox", []) or [])
    depth = getattr(detection, "depth_median", None)
    if len(bbox) < 4 or depth is None or image is None or not hasattr(image, "size"):
        return None
    depth = float(depth)
    max_depth_m = float(memory_config.get("MAX_DEPTH_M", 200.0))
    if not math.isfinite(depth) or depth <= 0.0 or depth > max_depth_m:
        return None

    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return None
    center_x = (float(bbox[0]) + float(bbox[2])) * 0.5
    center_y = (float(bbox[1]) + float(bbox[3])) * 0.5
    return project_image_depth_world(
        pixel_x=center_x,
        pixel_y=center_y,
        depth_m=depth,
        image_size=(width, height),
        camera=str(getattr(detection, "camera", "front") or "front"),
        observer_world=observer_world,
        observer_yaw_deg=observer_yaw_deg,
        memory_config=memory_config,
        sim_config=sim_config,
    )


def project_image_depth_world(
    *,
    pixel_x: float,
    pixel_y: float,
    depth_m: float,
    image_size: Sequence[float],
    camera: str,
    observer_world: Sequence[float],
    observer_yaw_deg: float,
    memory_config: Optional[dict] = None,
    sim_config: Optional[dict] = None,
) -> list[float]:
    """Project one radial/planar image-depth sample into AirSim world NED."""
    memory_config = memory_config or {}
    sim_config = sim_config or {}
    width, height = float(image_size[0]), float(image_size[1])
    depth = float(depth_m)
    camera = str(camera or "front").strip().lower()
    front_fov = float(memory_config.get("FRONT_FOV_DEG", sim_config.get("FRONT_FOV", 90.0)))
    down_fov = float(memory_config.get("DOWN_FOV_DEG", sim_config.get("DOWN_FOV", 90.0)))
    fov_deg = down_fov if camera == "down" else front_fov
    fx = width / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
    fy = fx

    ray_camera = [1.0, (float(pixel_x) - width * 0.5) / fx, (float(pixel_y) - height * 0.5) / fy]
    depth_is_radial = str(memory_config.get("DEPTH_MODE", "radial")).strip().lower() != "planar"
    if depth_is_radial:
        norm = math.sqrt(sum(value * value for value in ray_camera))
        ray_camera = [value / max(norm, 1e-9) for value in ray_camera]
    point_camera = [value * depth for value in ray_camera]

    front_offset = point3(memory_config.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0]))
    down_offset = point3(memory_config.get("DOWN_CAMERA_OFFSET", [0.0, 0.0, 0.0]))
    if camera == "down":
        # AirSim 的 down_center 相机 pitch=-90deg：相机前方对应机体系 +Z(NED向下)。
        point_body = [-point_camera[2], point_camera[1], point_camera[0]]
        camera_offset = down_offset
    else:
        point_body = point_camera
        camera_offset = front_offset
    point_body = [point_body[i] + camera_offset[i] for i in range(3)]

    yaw = math.radians(float(observer_yaw_deg))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    observer = point3(observer_world)
    return [
        observer[0] + cos_yaw * point_body[0] - sin_yaw * point_body[1],
        observer[1] + sin_yaw * point_body[0] + cos_yaw * point_body[1],
        observer[2] + point_body[2],
    ]


def estimate_detection_surface_world(
    detection: Any,
    image: Any,
    observer_world: Sequence[float],
    observer_yaw_deg: float,
    *,
    memory_config: Optional[dict] = None,
    sim_config: Optional[dict] = None,
) -> list[list[float]]:
    """Project the bounded sparse depth samples attached to a detection."""
    if detection is None or image is None or not hasattr(image, "size"):
        return []
    samples = list(getattr(detection, "surface_depth_samples", None) or [])
    if not samples:
        point = estimate_detection_world(
            detection,
            image,
            observer_world,
            observer_yaw_deg,
            memory_config=memory_config,
            sim_config=sim_config,
        )
        return [point] if point is not None else []
    memory_config = memory_config or {}
    max_depth_m = float(memory_config.get("MAX_DEPTH_M", 200.0))
    width, height = float(image.size[0]), float(image.size[1])
    out = []
    for sample in samples:
        if not isinstance(sample, (list, tuple)) or len(sample) < 3:
            continue
        u, v, depth = float(sample[0]), float(sample[1]), float(sample[2])
        if not all(math.isfinite(value) for value in (u, v, depth)):
            continue
        if depth <= 0.0 or depth > max_depth_m:
            continue
        out.append(project_image_depth_world(
            pixel_x=u * width,
            pixel_y=v * height,
            depth_m=depth,
            image_size=(width, height),
            camera=str(getattr(detection, "camera", "front") or "front"),
            observer_world=observer_world,
            observer_yaw_deg=observer_yaw_deg,
            memory_config=memory_config,
            sim_config=sim_config,
        ))
    return out


def bounds_from_points(points: Iterable[Sequence[float]]) -> Optional[list[list[float]]]:
    valid = [point3(point) for point in points if point is not None and len(point) >= 3]
    if not valid:
        return None
    return [
        [min(point[axis] for point in valid) for axis in range(3)],
        [max(point[axis] for point in valid) for axis in range(3)],
    ]


def valid_bounds(bounds: Any) -> bool:
    return bool(
        isinstance(bounds, (list, tuple))
        and len(bounds) >= 2
        and isinstance(bounds[0], (list, tuple))
        and isinstance(bounds[1], (list, tuple))
        and len(bounds[0]) >= 3
        and len(bounds[1]) >= 3
    )


def nearest_point_on_bounds(point: Sequence[float], bounds: Sequence[Sequence[float]]) -> list[float]:
    current = point3(point)
    if not valid_bounds(bounds):
        return current
    lower, upper = point3(bounds[0]), point3(bounds[1])
    return [max(lower[i], min(upper[i], current[i])) for i in range(3)]


def distance_to_bounds(point: Sequence[float], bounds: Sequence[Sequence[float]]) -> float:
    nearest = nearest_point_on_bounds(point, bounds)
    return distance3(point, nearest)


def horizontal_distance_to_bounds(point: Sequence[float], bounds: Sequence[Sequence[float]]) -> float:
    if not valid_bounds(bounds):
        return 0.0
    nearest = nearest_point_on_bounds(point, bounds)
    dx = float(point[0]) - nearest[0]
    dy = float(point[1]) - nearest[1]
    return math.sqrt(dx * dx + dy * dy)


def vertical_distance_to_bounds(point: Sequence[float], bounds: Sequence[Sequence[float]]) -> float:
    if not valid_bounds(bounds):
        return 0.0
    nearest = nearest_point_on_bounds(point, bounds)
    return abs(float(point[2]) - nearest[2])


def instance_surface_bounds(instance: Any) -> Optional[list[list[float]]]:
    bounds = getattr(instance, "surface_bounds_world", None)
    if valid_bounds(bounds):
        return [point3(bounds[0]), point3(bounds[1])]
    points = getattr(instance, "surface_points_world", None) or []
    return bounds_from_points(points)


def instance_surface_patches(instance: Any) -> list[list[list[float]]]:
    patches = []
    for raw_patch in list(getattr(instance, "surface_patches_world", None) or []):
        patch = [
            point3(point)
            for point in list(raw_patch or [])
            if point is not None and len(point) >= 3
        ]
        if patch:
            patches.append(patch)
    return patches


def instance_surface_sample_points(instance: Any) -> list[list[float]]:
    """Return the finite surface samples retained for one target instance.

    Completion uses these observed samples directly.  It deliberately does not
    use ``surface_bounds_world`` or a filled/interpolated patch because those
    can represent unobserved empty space between separate observations.
    """
    points: list[list[float]] = []
    for raw_point in list(getattr(instance, "surface_points_world", None) or []):
        if raw_point is not None and len(raw_point) >= 3:
            try:
                value = point3(raw_point)
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(v) for v in value):
                points.append(value)
    if points:
        return points
    # Older/hand-created memories may only contain patches.  Their actual
    # samples are still valid point evidence, while bounds remain excluded.
    for patch in instance_surface_patches(instance):
        points.extend(patch)
    return points


def nearest_instance_surface_sample_point(point: Sequence[float], instance: Any) -> list[float]:
    """Return the observed surface sample nearest in the horizontal plane."""
    samples = instance_surface_sample_points(instance)
    if not samples:
        return point3(getattr(instance, "target_world", [0.0, 0.0, 0.0]))
    return min(samples, key=lambda candidate: horizontal_distance(point, candidate))


def horizontal_distance_to_instance_surface_samples(point: Sequence[float], instance: Any) -> float:
    """Measure XY distance to the nearest observed surface sample, ignoring Z."""
    samples = instance_surface_sample_points(instance)
    if not samples:
        return horizontal_distance(point, getattr(instance, "target_world", [0.0, 0.0, 0.0]))
    return min(horizontal_distance(point, candidate) for candidate in samples)


def has_surface_geometry(instance: Any) -> bool:
    return bool(instance_surface_sample_points(instance) or instance_surface_bounds(instance) is not None) and int(
        getattr(instance, "surface_observation_count", 0) or 0
    ) > 0


def has_surface_samples(instance: Any) -> bool:
    """Return whether the instance has actual retained surface samples.

    Bounds are useful for legacy navigation and diagnostics, but they are not
    sufficient evidence for the large-target XY arrival shortcut.
    """
    return bool(instance_surface_sample_points(instance)) and int(
        getattr(instance, "surface_observation_count", 0) or 0
    ) > 0


def nearest_instance_surface_point(point: Sequence[float], instance: Any) -> list[float]:
    patches = instance_surface_patches(instance)
    if patches:
        candidates = [_nearest_point_on_surface_patch(point, patch) for patch in patches]
        return min(candidates, key=lambda candidate: distance3(point, candidate))
    bounds = instance_surface_bounds(instance)
    if bounds is not None:
        return nearest_point_on_bounds(point, bounds)
    return point3(getattr(instance, "target_world", [0.0, 0.0, 0.0]))


def distance_to_instance_geometry(point: Sequence[float], instance: Any) -> float:
    return distance3(point, nearest_instance_surface_point(point, instance))


def horizontal_distance_to_instance_geometry(point: Sequence[float], instance: Any) -> float:
    return horizontal_distance(point, nearest_instance_surface_point(point, instance))


def vertical_distance_to_instance_geometry(point: Sequence[float], instance: Any) -> float:
    target = nearest_instance_surface_point(point, instance)
    return abs(float(point[2]) - float(target[2]))


def _nearest_point_on_surface_patch(point: Sequence[float], patch: Sequence[Sequence[float]]) -> list[float]:
    """Project onto one finite planar patch without filling gaps to other patches."""
    points = [point3(value) for value in patch if value is not None and len(value) >= 3]
    if not points:
        return point3(point)
    if len(points) == 1:
        return points[0]
    try:
        import numpy as np

        values = np.asarray(points, dtype=float)
        center = values.mean(axis=0)
        centered = values - center
        _u, singular, vh = np.linalg.svd(centered, full_matrices=True)
        if len(singular) < 2 or float(singular[1]) <= 1e-6:
            return _nearest_point_on_segments(point, points)

        basis_u = vh[0]
        basis_v = vh[1]
        normal = vh[2]
        query = np.asarray(point3(point), dtype=float)
        relative = query - center
        projected = query - float(relative @ normal) * normal
        projected_relative = projected - center
        patch_u = centered @ basis_u
        patch_v = centered @ basis_v
        query_u = float(projected_relative @ basis_u)
        query_v = float(projected_relative @ basis_v)
        clamped_u = min(max(query_u, float(patch_u.min())), float(patch_u.max()))
        clamped_v = min(max(query_v, float(patch_v.min())), float(patch_v.max()))
        nearest = center + clamped_u * basis_u + clamped_v * basis_v
        return [float(nearest[0]), float(nearest[1]), float(nearest[2])]
    except Exception:
        # Missing/degenerate linear algebra falls back conservatively to
        # observed points and line segments, never to a filled 3-D box.
        return _nearest_point_on_segments(point, points)


def _nearest_point_on_segments(point: Sequence[float], points: Sequence[Sequence[float]]) -> list[float]:
    query = point3(point)
    best = min((point3(value) for value in points), key=lambda value: distance3(query, value))
    best_distance = distance3(query, best)
    for index, start in enumerate(points):
        a = point3(start)
        for end in points[index + 1:]:
            b = point3(end)
            ab = [b[axis] - a[axis] for axis in range(3)]
            denom = sum(value * value for value in ab)
            if denom <= 1e-9:
                continue
            t = sum((query[axis] - a[axis]) * ab[axis] for axis in range(3)) / denom
            t = max(0.0, min(1.0, t))
            candidate = [a[axis] + t * ab[axis] for axis in range(3)]
            candidate_distance = distance3(query, candidate)
            if candidate_distance < best_distance:
                best = candidate
                best_distance = candidate_distance
    return best


def footprint_radius_from_detection(
    detection: Any,
    image: Any,
    *,
    memory_config: Optional[dict] = None,
    sim_config: Optional[dict] = None,
) -> float:
    """Estimate a small horizontal footprint radius from bbox size and depth."""
    memory_config = memory_config or {}
    sim_config = sim_config or {}
    default_radius = float(memory_config.get("DEFAULT_FOOTPRINT_RADIUS_M", 1.5))
    if detection is None or not getattr(detection, "bbox", None) or image is None or not hasattr(image, "size"):
        return default_radius
    depth = getattr(detection, "depth_median", None)
    if depth is None:
        return default_radius
    depth = float(depth)
    if not math.isfinite(depth) or depth <= 0.0:
        return default_radius
    width, height = float(image.size[0]), float(image.size[1])
    if width <= 1.0 or height <= 1.0:
        return default_radius
    x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
    span_x = max(0.0, min(width, x2) - max(0.0, x1)) / width
    span_y = max(0.0, min(height, y2) - max(0.0, y1)) / height
    camera = str(getattr(detection, "camera", "front") or "front").strip().lower()
    fov = float(
        memory_config.get(
            "DOWN_FOV_DEG" if camera == "down" else "FRONT_FOV_DEG",
            sim_config.get("DOWN_FOV" if camera == "down" else "FRONT_FOV", 90.0),
        )
    )
    visible_width_m = 2.0 * depth * math.tan(math.radians(fov) * 0.5)
    approx_w = span_x * visible_width_m
    approx_h = span_y * visible_width_m
    radius = 0.5 * math.sqrt(approx_w * approx_w + approx_h * approx_h)
    return max(0.5, min(float(memory_config.get("MAX_FOOTPRINT_RADIUS_M", 8.0)), radius or default_radius))


def world_to_body(
    world_point: Sequence[float],
    current_world: Sequence[float],
    yaw_deg: float,
) -> list[float]:
    """Convert a world point into current body-frame coordinates."""
    dx = float(world_point[0]) - float(current_world[0])
    dy = float(world_point[1]) - float(current_world[1])
    dz = float(world_point[2]) - float(current_world[2])
    yaw = math.radians(float(yaw_deg))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return [
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        dz,
    ]


def bearing_yaw_deg(current_world: Sequence[float], target_world: Sequence[float]) -> float:
    dx = float(target_world[0]) - float(current_world[0])
    dy = float(target_world[1]) - float(current_world[1])
    return math.degrees(math.atan2(dy, dx))
