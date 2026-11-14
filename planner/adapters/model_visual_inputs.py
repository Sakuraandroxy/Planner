"""Prepare model-only depth images for registered, calibrated pinhole cameras.

Input is an Observation with separate RGB/depth intrinsics and a depth image
in the original depth grid. The capture adapter must verify shared camera pose.
Output covers the RGB view; unsupported rays are black. Raw images, float depth
and calibration are never changed. No state, device calls or network requests.
"""
import math
from numbers import Integral

import numpy as np
from PIL import Image

from planner.domain.observation import CameraIntrinsics, Observation
from planner.errors import ObservationError


def _validate_intrinsics(k: CameraIntrinsics) -> None:
    if any(isinstance(v, bool) or not isinstance(v, Integral) or v <= 0 for v in (k.width, k.height)):
        raise ObservationError("model image dimensions must be positive integers")
    try:
        valid = (not any(isinstance(v, bool) for v in (k.fx, k.fy, k.cx, k.cy, k.horizontal_fov_deg)) and
                 all(math.isfinite(v) and v > 0 for v in (k.fx, k.fy)) and
                 all(math.isfinite(v) for v in (k.cx, k.cy, k.horizontal_fov_deg)) and
                 0 < k.horizontal_fov_deg < 180)
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise ObservationError("model visual inputs require valid camera intrinsics")


def _normalized_intrinsics(k: CameraIntrinsics) -> tuple[float, ...]:
    return k.fx/k.width, k.fy/k.height, k.cx/k.width, k.cy/k.height


def aligned_depth_image(observation: Observation) -> Image.Image:
    """Nearest-neighbor RGB-ray mapping of the existing depth visualization.

    Matching normalized intrinsics reuse the original image, regardless of
    resolution. Otherwise output uses RGB aspect and no more depth columns
    than the original depth/RGB grids. Invalid calibration raises
    ObservationError before any model request.
    """
    rgb, depth = observation.intrinsics, observation.depth_intrinsics
    if depth is None:
        raise ObservationError("model visual inputs require calibrated depth intrinsics")
    for k in (rgb, depth):
        _validate_intrinsics(k)
    if (observation.rgb.size != (rgb.width, rgb.height) or
            observation.depth_image.size != (depth.width, depth.height) or
            np.shape(observation.depth_meters) != (depth.height, depth.width)):
        raise ObservationError("model images do not match their calibrated dimensions")
    if np.allclose(_normalized_intrinsics(rgb), _normalized_intrinsics(depth), rtol=1e-9, atol=1e-9):
        return observation.depth_image

    width = min(rgb.width, depth.width)
    height = max(1, round(width*rgb.height/rgb.width))
    xs = ((np.arange(width)/width*rgb.width-rgb.cx)/rgb.fx)*depth.fx+depth.cx
    ys = ((np.arange(height)/height*rgb.height-rgb.cy)/rgb.fy)*depth.fy+depth.cy
    valid_x = np.isfinite(xs) & (xs >= 0) & (xs < depth.width)
    valid_y = np.isfinite(ys) & (ys >= 0) & (ys < depth.height)
    ix = np.clip(np.rint(np.where(valid_x, xs, 0)), 0, depth.width-1).astype(np.intp)
    iy = np.clip(np.rint(np.where(valid_y, ys, 0)), 0, depth.height-1).astype(np.intp)
    source = np.asarray(observation.depth_image.convert("RGB"))
    sampled = source[iy[:, None], ix[None, :]]
    output = np.zeros((height, width, 3), dtype=np.uint8)
    supported = valid_y[:, None] & valid_x[None, :]
    output[supported] = sampled[supported]
    return Image.fromarray(output)
