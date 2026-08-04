"""Geometry helpers for lightweight target memory."""

from __future__ import annotations

import math
from typing import Any, Optional, Sequence


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
    camera = str(getattr(detection, "camera", "front") or "front").strip().lower()
    front_fov = float(memory_config.get("FRONT_FOV_DEG", sim_config.get("FRONT_FOV", 90.0)))
    down_fov = float(memory_config.get("DOWN_FOV_DEG", sim_config.get("DOWN_FOV", 90.0)))
    fov_deg = down_fov if camera == "down" else front_fov
    fx = width / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
    fy = fx

    ray_camera = [1.0, (center_x - width * 0.5) / fx, (center_y - height * 0.5) / fy]
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
