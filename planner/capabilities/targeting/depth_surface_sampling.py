"""Read exactly one calibrated raw-depth pixel, without neighborhood processing."""
from dataclasses import dataclass
import math

import numpy as np

from planner.domain.observation import CameraIntrinsics


@dataclass(frozen=True)
class SurfaceDepthSample:
    valid: bool
    reason: str
    depth_m: float | None = None


def sample_surface_depth(
    depth: np.ndarray,
    pixel: tuple[float, float],
    bbox: tuple[float, float, float, float],
    rgb_intrinsics: CameraIntrinsics,
    depth_intrinsics: CameraIntrinsics,
    minimum_m: float,
    maximum_m: float,
) -> SurfaceDepthSample:
    """Read one mapped pixel; invalid data is never filled from its neighbors."""
    u, v = pixel
    k, rgb = depth_intrinsics, rgb_intrinsics
    if not (math.isfinite(u) and math.isfinite(v) and 0 <= u < k.width and 0 <= v < k.height):
        return SurfaceDepthSample(False, "mode=direct_pixel; target ray is outside depth coverage")
    x, y = int(u), int(v)
    prefix = f"mode=direct_pixel; pixel=[{x},{y}]"
    sample_u = ((x-k.cx)/k.fx*rgb.fx+rgb.cx)/rgb.width
    sample_v = ((y-k.cy)/k.fy*rgb.fy+rgb.cy)/rgb.height
    if not (bbox[0] <= sample_u <= bbox[2] and bbox[1] <= sample_v <= bbox[3]):
        return SurfaceDepthSample(False, prefix+"; sampled pixel is outside target bbox")
    value = float(depth[y, x])
    if not (math.isfinite(value) and value > 0 and minimum_m <= value <= maximum_m):
        return SurfaceDepthSample(False, prefix+"; depth is invalid")
    return SurfaceDepthSample(True, prefix+f"; depth_m={value:.6g}", value)
