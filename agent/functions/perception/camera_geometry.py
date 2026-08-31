"""Camera-model independent projection between AirSim optical and world frames.

Coordinate convention:

* AirSim world is NED: +X north, +Y east, +Z down.
* Camera optical coordinates are +X forward, +Y image-right, +Z image-down.
* ``rotation_camera_to_world`` and ``camera_position_world`` come from the
  exact Camera API response that produced the pixels.

No function in this module infers orientation from a camera name.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np

from sim.camera_frames import CameraFrame, CameraIntrinsics, WorldRay


def _vec3(value: Sequence[float]) -> np.ndarray:
    array = np.asarray(list(value), dtype=float).reshape(-1)
    if array.size < 3:
        raise ValueError("expected a 3D vector")
    return array[:3]


def _rotation(value: Sequence[Sequence[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("expected a finite 3x3 rotation matrix")
    return matrix


def normalize(vector: Sequence[float]) -> np.ndarray:
    value = _vec3(vector)
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("cannot normalize a zero/non-finite vector")
    return value / norm


def camera_to_world(point_camera: Sequence[float], frame: CameraFrame, *, use_depth_pose: bool = False) -> list[float]:
    position, rotation = camera_pose(frame, use_depth_pose=use_depth_pose)
    point = position + rotation @ _vec3(point_camera)
    return point.astype(float).tolist()


def world_to_camera(point_world: Sequence[float], frame: CameraFrame, *, use_depth_pose: bool = False) -> list[float]:
    position, rotation = camera_pose(frame, use_depth_pose=use_depth_pose)
    point = rotation.T @ (_vec3(point_world) - position)
    return point.astype(float).tolist()


def camera_pose(frame: CameraFrame, *, use_depth_pose: bool = False) -> tuple[np.ndarray, np.ndarray]:
    if use_depth_pose and frame.depth_camera_position_world is not None:
        position = _vec3(frame.depth_camera_position_world)
        rotation_value = frame.depth_rotation_camera_to_world or frame.rotation_camera_to_world
    else:
        position = _vec3(frame.camera_position_world)
        rotation_value = frame.rotation_camera_to_world
    return position, _rotation(rotation_value)


def intrinsics_for_frame(frame: CameraFrame, *, depth: bool = False) -> CameraIntrinsics:
    intrinsics = frame.depth_intrinsics if depth else frame.rgb_intrinsics
    if intrinsics is None:
        kind = "depth" if depth else "RGB"
        raise ValueError(f"{kind} intrinsics unavailable for camera {frame.camera_id}")
    return intrinsics


def pixel_to_camera_ray(
    pixel_x: float,
    pixel_y: float,
    intrinsics: CameraIntrinsics,
    *,
    normalize_ray: bool = True,
) -> list[float]:
    ray = np.asarray(
        [
            1.0,
            (float(pixel_x) - float(intrinsics.cx)) / float(intrinsics.fx),
            (float(pixel_y) - float(intrinsics.cy)) / float(intrinsics.fy),
        ],
        dtype=float,
    )
    if normalize_ray:
        ray = normalize(ray)
    return ray.astype(float).tolist()


def pixel_to_world_ray(
    frame: CameraFrame,
    pixel_x: float,
    pixel_y: float,
    *,
    depth_intrinsics: bool = False,
    angular_uncertainty_deg: float = 0.0,
) -> WorldRay:
    intrinsics = intrinsics_for_frame(frame, depth=depth_intrinsics)
    ray_camera = np.asarray(pixel_to_camera_ray(pixel_x, pixel_y, intrinsics), dtype=float)
    position, rotation = camera_pose(frame, use_depth_pose=depth_intrinsics)
    direction = normalize(rotation @ ray_camera)
    timestamp = frame.depth_timestamp_ns if depth_intrinsics else frame.timestamp_ns
    return WorldRay(
        camera_id=frame.camera_id,
        capture_id=frame.capture_id,
        origin_world=tuple(float(value) for value in position),
        direction_world=tuple(float(value) for value in direction),
        timestamp_ns=int(timestamp or 0),
        angular_uncertainty_deg=max(0.0, float(angular_uncertainty_deg)),
    )


def pixel_depth_to_camera(
    pixel_x: float,
    pixel_y: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    *,
    depth_mode: str | None = None,
) -> list[float]:
    depth = float(depth_m)
    if not math.isfinite(depth) or depth <= 0.0:
        raise ValueError("depth must be finite and positive")
    mode = str(depth_mode or intrinsics.depth_mode or "radial").strip().lower()
    # Radial/range depth scales a unit bearing. AirSim DepthPerspective is
    # planar optical-axis depth and therefore scales [1, x/fx, y/fy] directly.
    normalize_ray = mode not in {"planar", "perspective", "optical_axis"}
    ray = np.asarray(
        pixel_to_camera_ray(pixel_x, pixel_y, intrinsics, normalize_ray=normalize_ray),
        dtype=float,
    )
    return (depth * ray).astype(float).tolist()


def pixel_depth_to_world(
    frame: CameraFrame,
    pixel_x: float,
    pixel_y: float,
    depth_m: float,
    *,
    pixels_use_rgb_intrinsics: bool = True,
    depth_mode: str | None = None,
) -> list[float]:
    intrinsics = intrinsics_for_frame(frame, depth=not pixels_use_rgb_intrinsics)
    if depth_mode is None and frame.depth_intrinsics is not None:
        depth_mode = frame.depth_intrinsics.depth_mode
    point_camera = pixel_depth_to_camera(pixel_x, pixel_y, depth_m, intrinsics, depth_mode=depth_mode)
    return camera_to_world(point_camera, frame, use_depth_pose=not pixels_use_rgb_intrinsics)


def project_world_to_pixel(
    frame: CameraFrame,
    point_world: Sequence[float],
    *,
    depth_intrinsics: bool = False,
    require_in_frame: bool = False,
) -> Optional[tuple[float, float, float]]:
    intrinsics = intrinsics_for_frame(frame, depth=depth_intrinsics)
    x, y, z = world_to_camera(point_world, frame, use_depth_pose=depth_intrinsics)
    if not math.isfinite(x) or x <= 1e-6:
        return None
    pixel_x = float(intrinsics.cx) + float(intrinsics.fx) * y / x
    pixel_y = float(intrinsics.cy) + float(intrinsics.fy) * z / x
    if require_in_frame and not (0.0 <= pixel_x < intrinsics.width and 0.0 <= pixel_y < intrinsics.height):
        return None
    return float(pixel_x), float(pixel_y), float(x)


def depth_map_to_world_points(
    frame: CameraFrame,
    depth_map=None,
    *,
    stride: int = 1,
    min_depth_m: float = 0.05,
    max_depth_m: float = 200.0,
    pixel_bounds: Sequence[int] | None = None,
) -> np.ndarray:
    """Back-project a sampled depth map into world NED points."""
    points, _pixels, _depths = depth_map_to_world_samples(
        frame,
        depth_map,
        stride=stride,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
        pixel_bounds=pixel_bounds,
    )
    return points


def depth_map_to_world_samples(
    frame: CameraFrame,
    depth_map=None,
    *,
    stride: int = 1,
    min_depth_m: float = 0.05,
    max_depth_m: float = 200.0,
    pixel_bounds: Sequence[int] | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return aligned ``(world_points, pixels_uv, radial_depths)`` arrays."""
    intrinsics = intrinsics_for_frame(frame, depth=True)
    depth = np.asarray(frame.depth if depth_map is None else depth_map, dtype=float)
    if depth.ndim != 2:
        raise ValueError("depth map must be HxW")
    height, width = depth.shape
    if (height, width) != (intrinsics.height, intrinsics.width):
        raise ValueError(
            f"depth shape {depth.shape} does not match intrinsics {(intrinsics.height, intrinsics.width)}"
        )
    if pixel_bounds is None:
        x1, y1, x2, y2 = 0, 0, width, height
    else:
        x1, y1, x2, y2 = [int(value) for value in pixel_bounds[:4]]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(width, x2), min(height, y2)
    step = max(1, int(stride))
    rows, columns = np.mgrid[y1:y2:step, x1:x2:step]
    values = depth[y1:y2:step, x1:x2:step]
    valid = np.isfinite(values) & (values >= float(min_depth_m)) & (values <= float(max_depth_m))
    if not valid.any():
        return (
            np.empty((0, 3), dtype=float),
            np.empty((0, 2), dtype=float),
            np.empty((0,), dtype=float),
        )
    u = columns[valid].astype(float)
    v = rows[valid].astype(float)
    distances = values[valid].astype(float)
    rays = np.stack(
        [
            np.ones_like(u),
            (u - float(intrinsics.cx)) / float(intrinsics.fx),
            (v - float(intrinsics.cy)) / float(intrinsics.fy),
        ],
        axis=1,
    )
    mode = str(intrinsics.depth_mode or "radial").strip().lower()
    if mode not in {"planar", "perspective", "optical_axis"}:
        norms = np.linalg.norm(rays, axis=1, keepdims=True)
        rays = rays / np.maximum(norms, 1e-12)
    points_camera = rays * distances[:, None]
    position, rotation = camera_pose(frame, use_depth_pose=True)
    points_world = points_camera @ rotation.T + position[None, :]
    pixels = np.stack([u, v], axis=1)
    return points_world, pixels, distances


def world_to_navigation(
    point_world: Sequence[float],
    navigation_origin_world: Sequence[float],
    navigation_yaw_deg: float,
) -> list[float]:
    delta = _vec3(point_world) - _vec3(navigation_origin_world)
    yaw = math.radians(float(navigation_yaw_deg))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        cosine * delta[0] + sine * delta[1],
        -sine * delta[0] + cosine * delta[1],
        float(delta[2]),
    ]


def navigation_to_world(
    point_navigation: Sequence[float],
    navigation_origin_world: Sequence[float],
    navigation_yaw_deg: float,
) -> list[float]:
    point = _vec3(point_navigation)
    origin = _vec3(navigation_origin_world)
    yaw = math.radians(float(navigation_yaw_deg))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        float(origin[0] + cosine * point[0] - sine * point[1]),
        float(origin[1] + sine * point[0] + cosine * point[1]),
        float(origin[2] + point[2]),
    ]


def triangulate_world_rays(
    rays: Iterable[WorldRay],
    *,
    minimum_baseline_m: float = 0.25,
    minimum_angle_deg: float = 1.0,
) -> Optional[tuple[list[float], float]]:
    """Least-squares intersection of two or more world rays.

    Returns ``(point_world, residual_m)`` or ``None`` for degenerate geometry.
    """
    values = list(rays)
    if len(values) < 2:
        return None
    origins = [np.asarray(ray.origin_world, dtype=float) for ray in values]
    directions = [normalize(ray.direction_world) for ray in values]
    maximum_baseline = max(float(np.linalg.norm(a - b)) for i, a in enumerate(origins) for b in origins[i + 1 :])
    if maximum_baseline < float(minimum_baseline_m):
        return None
    maximum_angle = 0.0
    for index, first in enumerate(directions):
        for second in directions[index + 1 :]:
            dot = float(np.clip(abs(first @ second), 0.0, 1.0))
            maximum_angle = max(maximum_angle, math.degrees(math.acos(dot)))
    if maximum_angle < float(minimum_angle_deg):
        return None
    identity = np.eye(3)
    lhs = np.zeros((3, 3), dtype=float)
    rhs = np.zeros(3, dtype=float)
    for origin, direction in zip(origins, directions):
        projection = identity - np.outer(direction, direction)
        lhs += projection
        rhs += projection @ origin
    try:
        point = np.linalg.solve(lhs, rhs)
    except np.linalg.LinAlgError:
        point = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    residuals = [float(np.linalg.norm(np.cross(point - origin, direction))) for origin, direction in zip(origins, directions)]
    return point.astype(float).tolist(), float(np.mean(residuals))


@dataclass(frozen=True)
class ProjectionComparison:
    legacy_world: tuple[float, float, float]
    unified_world: tuple[float, float, float]
    error_m: float
    camera_id: str
    capture_id: str


class GeometryShadowTracker:
    """Thread-safe aggregate of old/new projection disagreement."""

    def __init__(self):
        self._lock = threading.Lock()
        self._count = 0
        self._sum_error_m = 0.0
        self._max_error_m = 0.0
        self._last: Optional[ProjectionComparison] = None

    def observe(
        self,
        legacy_world: Sequence[float],
        unified_world: Sequence[float],
        frame: CameraFrame,
    ) -> ProjectionComparison:
        legacy = _vec3(legacy_world)
        unified = _vec3(unified_world)
        comparison = ProjectionComparison(
            legacy_world=tuple(float(value) for value in legacy),
            unified_world=tuple(float(value) for value in unified),
            error_m=float(np.linalg.norm(legacy - unified)),
            camera_id=frame.camera_id,
            capture_id=frame.capture_id,
        )
        with self._lock:
            self._count += 1
            self._sum_error_m += comparison.error_m
            self._max_error_m = max(self._max_error_m, comparison.error_m)
            self._last = comparison
        return comparison

    def summary(self) -> dict:
        with self._lock:
            return {
                "count": self._count,
                "mean_error_m": self._sum_error_m / self._count if self._count else 0.0,
                "max_error_m": self._max_error_m,
                "last": self._last,
            }


geometry_shadow_tracker = GeometryShadowTracker()


def attach_detection_camera_context(detection, image, frame: CameraFrame | None = None):
    """Attach exact frame provenance and an RGB-only world ray to a detection."""
    if detection is None:
        return detection
    camera_frame = frame or getattr(image, "camera_frame", None)
    if camera_frame is None:
        return detection
    detection.camera_frame = camera_frame
    detection.camera_id = camera_frame.camera_id
    detection.capture_id = camera_frame.capture_id
    bbox = list(getattr(detection, "bbox", None) or [])
    if len(bbox) >= 4 and camera_frame.rgb_intrinsics is not None:
        center_x = 0.5 * (float(bbox[0]) + float(bbox[2]))
        center_y = 0.5 * (float(bbox[1]) + float(bbox[3]))
        try:
            detection.world_ray = pixel_to_world_ray(camera_frame, center_x, center_y)
        except (TypeError, ValueError):
            detection.world_ray = None
    return detection
