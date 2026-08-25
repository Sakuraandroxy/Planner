"""Depth-only roof-plane estimation for ``above`` navigation.

The detector is intentionally not part of this module.  A large roof can fill
the complete down-view and therefore produce no useful GroundingDINO box.  We
instead back-project a sparse grid from AirSim ``DepthPerspective`` into world
NED coordinates, find the horizontal plane supporting the image centre, and
return bounded geometry that MissionMemory can associate with its locked
building instance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence


@dataclass(frozen=True)
class RoofPlaneEstimate:
    valid: bool
    reason: str
    roof_z_world: Optional[float] = None
    center_world: Optional[list[float]] = None
    center_depth_m: Optional[float] = None
    valid_ratio: float = 0.0
    coverage_ratio: float = 0.0
    center_support_ratio: float = 0.0
    z_mad_m: Optional[float] = None
    normal_error_deg: Optional[float] = None
    confidence: float = 0.0
    full_frame: bool = False
    sample_points_world: list[list[float]] = field(default_factory=list)
    bounds_world: Optional[list[list[float]]] = None

    def to_summary_dict(self) -> dict:
        return {
            "valid": bool(self.valid),
            "reason": self.reason,
            "roof_z_world": None if self.roof_z_world is None else round(float(self.roof_z_world), 2),
            "center_world": (
                None
                if self.center_world is None
                else [round(float(value), 2) for value in self.center_world[:3]]
            ),
            "center_depth_m": None if self.center_depth_m is None else round(float(self.center_depth_m), 2),
            "valid_ratio": round(float(self.valid_ratio), 3),
            "coverage_ratio": round(float(self.coverage_ratio), 3),
            "center_support_ratio": round(float(self.center_support_ratio), 3),
            "z_mad_m": None if self.z_mad_m is None else round(float(self.z_mad_m), 3),
            "normal_error_deg": (
                None if self.normal_error_deg is None else round(float(self.normal_error_deg), 2)
            ),
            "confidence": round(float(self.confidence), 3),
            "full_frame": bool(self.full_frame),
            "samples": len(self.sample_points_world),
        }


def estimate_down_roof_plane(
    down_depth_meters: Any,
    observer_world: Sequence[float],
    observer_yaw_deg: float,
    *,
    config: Optional[dict] = None,
    sim_config: Optional[dict] = None,
) -> RoofPlaneEstimate:
    """Estimate the horizontal plane underneath the down camera.

    AirSim ``DepthPerspective`` is radial depth.  Every sampled pixel is first
    converted into a unit camera ray and then rotated from a pitch=-90 degree
    down camera into body/world NED.  Plane support is selected around the
    robust world-Z value in the central image region; this makes the estimator
    prefer what is directly under the aircraft when roof and ground are both
    visible.
    """

    config = dict(config or {})
    sim_config = dict(sim_config or {})
    try:
        import numpy as np

        depth = np.asarray(down_depth_meters, dtype=float)
        if depth.ndim != 2 or depth.size == 0:
            return RoofPlaneEstimate(False, "down_depth_not_2d")
        height, width = int(depth.shape[0]), int(depth.shape[1])
    except Exception:
        return RoofPlaneEstimate(False, "down_depth_unavailable")

    stride = max(1, int(config.get("ABOVE_ROOF_DEPTH_STRIDE", 4)))
    min_depth = max(0.05, float(config.get("ABOVE_ROOF_MIN_DEPTH_M", 0.8)))
    max_depth = max(min_depth, float(config.get("ABOVE_ROOF_MAX_DEPTH_M", 120.0)))
    horizontal_fov_deg = float(
        config.get("DOWN_FOV_DEG", sim_config.get("DOWN_DEPTH_FOV", sim_config.get("DOWN_FOV", 90.0)))
    )
    fx = float(width) / (2.0 * math.tan(math.radians(horizontal_fov_deg) * 0.5))
    fy = fx
    offset_raw = config.get("DOWN_CAMERA_OFFSET", sim_config.get("DOWN_CAMERA_OFFSET", [0.0, 0.0, 0.0]))
    offset = _point3(offset_raw)
    observer = _point3(observer_world)
    yaw = math.radians(float(observer_yaw_deg))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    rows = list(range(stride // 2, height, stride))
    cols = list(range(stride // 2, width, stride))
    total_samples = max(1, len(rows) * len(cols))
    points: list[list[float]] = []
    pixels: list[tuple[int, int]] = []
    depths: list[float] = []
    for py in rows:
        for px in cols:
            value = float(depth[py, px])
            if not math.isfinite(value) or value < min_depth or value > max_depth:
                continue
            ray = [1.0, (float(px) + 0.5 - width * 0.5) / fx, (float(py) + 0.5 - height * 0.5) / fy]
            norm = math.sqrt(sum(component * component for component in ray))
            ray = [component / max(norm, 1e-9) for component in ray]
            camera = [component * value for component in ray]
            body = [
                offset[0] - camera[2],
                offset[1] + camera[1],
                offset[2] + camera[0],
            ]
            world = [
                observer[0] + cos_yaw * body[0] - sin_yaw * body[1],
                observer[1] + sin_yaw * body[0] + cos_yaw * body[1],
                observer[2] + body[2],
            ]
            points.append(world)
            pixels.append((px, py))
            depths.append(value)

    valid_ratio = float(len(points)) / float(total_samples)
    min_valid_ratio = float(config.get("ABOVE_ROOF_MIN_VALID_RATIO", 0.35))
    if not points or valid_ratio < min_valid_ratio:
        return RoofPlaneEstimate(
            False,
            "insufficient_valid_down_depth",
            valid_ratio=valid_ratio,
        )

    import numpy as np

    point_array = np.asarray(points, dtype=float)
    pixel_array = np.asarray(pixels, dtype=float)
    depth_array = np.asarray(depths, dtype=float)
    centre_half_span = max(0.05, min(0.45, float(config.get("ABOVE_ROOF_CENTER_FRACTION", 0.22))))
    centre_mask = (
        (np.abs((pixel_array[:, 0] + 0.5) / max(float(width), 1.0) - 0.5) <= centre_half_span)
        & (np.abs((pixel_array[:, 1] + 0.5) / max(float(height), 1.0) - 0.5) <= centre_half_span)
    )
    centre_z = point_array[centre_mask, 2]
    if centre_z.size < max(4, int(config.get("ABOVE_ROOF_MIN_CENTER_SAMPLES", 12))):
        return RoofPlaneEstimate(
            False,
            "insufficient_center_depth_support",
            valid_ratio=valid_ratio,
        )

    anchor_z = float(np.median(centre_z))
    plane_tolerance = max(0.15, float(config.get("ABOVE_ROOF_PLANE_TOLERANCE_M", 0.9)))
    support_mask = np.abs(point_array[:, 2] - anchor_z) <= plane_tolerance
    support = point_array[support_mask]
    coverage_ratio = float(support.shape[0]) / max(float(point_array.shape[0]), 1.0)
    center_support_ratio = float(np.count_nonzero(support_mask & centre_mask)) / max(
        float(np.count_nonzero(centre_mask)),
        1.0,
    )
    min_coverage = float(config.get("ABOVE_ROOF_MIN_COVERAGE_RATIO", 0.20))
    min_center_support = float(config.get("ABOVE_ROOF_MIN_CENTER_SUPPORT_RATIO", 0.65))
    if support.shape[0] < 6 or coverage_ratio < min_coverage or center_support_ratio < min_center_support:
        return RoofPlaneEstimate(
            False,
            "down_center_not_supported_by_one_plane",
            roof_z_world=anchor_z,
            valid_ratio=valid_ratio,
            coverage_ratio=coverage_ratio,
            center_support_ratio=center_support_ratio,
        )

    roof_z = float(np.median(support[:, 2]))
    z_mad = float(np.median(np.abs(support[:, 2] - roof_z)))
    max_z_mad = max(0.05, float(config.get("ABOVE_ROOF_MAX_Z_MAD_M", 0.45)))
    if not math.isfinite(z_mad) or z_mad > max_z_mad:
        return RoofPlaneEstimate(
            False,
            "roof_plane_z_too_noisy",
            roof_z_world=roof_z,
            valid_ratio=valid_ratio,
            coverage_ratio=coverage_ratio,
            center_support_ratio=center_support_ratio,
            z_mad_m=z_mad,
        )

    normal_error = _plane_normal_error_deg(support)
    max_normal_error = float(config.get("ABOVE_ROOF_MAX_NORMAL_ERROR_DEG", 15.0))
    if normal_error is None or normal_error > max_normal_error:
        return RoofPlaneEstimate(
            False,
            "dominant_down_surface_not_horizontal",
            roof_z_world=roof_z,
            valid_ratio=valid_ratio,
            coverage_ratio=coverage_ratio,
            center_support_ratio=center_support_ratio,
            z_mad_m=z_mad,
            normal_error_deg=normal_error,
        )

    # Use the centre-most supported sample for the nadir/range observation.
    centre_distance = (
        ((pixel_array[:, 0] + 0.5) / max(float(width), 1.0) - 0.5) ** 2
        + ((pixel_array[:, 1] + 0.5) / max(float(height), 1.0) - 0.5) ** 2
    )
    supported_indices = np.flatnonzero(support_mask)
    centre_index = int(supported_indices[int(np.argmin(centre_distance[support_mask]))])
    centre_world = [float(value) for value in point_array[centre_index, :3]]
    centre_depth = float(depth_array[centre_index])

    max_points = max(4, int(config.get("ABOVE_ROOF_MAX_PLANE_SAMPLES", 64)))
    if support.shape[0] > max_points:
        indices = np.linspace(0, support.shape[0] - 1, max_points).round().astype(int)
        bounded_support = support[indices]
    else:
        bounded_support = support
    sample_points = [[float(value) for value in row[:3]] for row in bounded_support]
    bounds = _bounds_from_points(sample_points)

    full_frame_ratio = float(config.get("ABOVE_ROOF_FULL_FRAME_COVERAGE_RATIO", 0.65))
    full_frame = bool(coverage_ratio >= full_frame_ratio and center_support_ratio >= 0.9)
    normal_score = max(0.0, 1.0 - normal_error / max(max_normal_error, 1e-6))
    noise_score = max(0.0, 1.0 - z_mad / max(max_z_mad, 1e-6))
    confidence = max(
        0.0,
        min(
            0.99,
            0.25 * min(valid_ratio / max(min_valid_ratio, 1e-6), 1.0)
            + 0.30 * min(coverage_ratio / max(full_frame_ratio, min_coverage, 1e-6), 1.0)
            + 0.20 * center_support_ratio
            + 0.15 * normal_score
            + 0.10 * noise_score,
        ),
    )
    return RoofPlaneEstimate(
        True,
        "horizontal_plane_under_drone",
        roof_z_world=roof_z,
        center_world=centre_world,
        center_depth_m=centre_depth,
        valid_ratio=valid_ratio,
        coverage_ratio=coverage_ratio,
        center_support_ratio=center_support_ratio,
        z_mad_m=z_mad,
        normal_error_deg=normal_error,
        confidence=confidence,
        full_frame=full_frame,
        sample_points_world=sample_points,
        bounds_world=bounds,
    )


def _plane_normal_error_deg(points) -> Optional[float]:
    try:
        import numpy as np

        values = np.asarray(points, dtype=float)
        if values.ndim != 2 or values.shape[0] < 3 or values.shape[1] < 3:
            return None
        centred = values[:, :3] - values[:, :3].mean(axis=0)
        _u, _s, vh = np.linalg.svd(centred, full_matrices=False)
        normal = vh[-1]
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-9:
            return None
        vertical_alignment = max(0.0, min(1.0, abs(float(normal[2])) / norm))
        return math.degrees(math.acos(vertical_alignment))
    except Exception:
        return None


def _point3(value: Sequence[float]) -> list[float]:
    if value is None or len(value) < 3:
        return [0.0, 0.0, 0.0]
    return [float(value[0]), float(value[1]), float(value[2])]


def _bounds_from_points(points: list[list[float]]) -> Optional[list[list[float]]]:
    if not points:
        return None
    return [
        [min(float(point[axis]) for point in points) for axis in range(3)],
        [max(float(point[axis]) for point in points) for axis in range(3)],
    ]


__all__ = ["RoofPlaneEstimate", "estimate_down_roof_plane"]
