"""Prepare model-only depth images for registered, calibrated pinhole cameras.

Input is an Observation whose depth visualisation is already registered to the
RGB camera. The CARLA rig renders RGB and depth from one parentless camera, so
both share intrinsics. Output covers the RGB view; unsupported rays are black.
Raw images, float depth and calibration are never changed. No state, device
calls or network requests.
"""
import math
from numbers import Integral

import numpy as np

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


def aligned_depth_image(observation: Observation) -> np.ndarray:
    """Return the registered depth visualisation in the RGB view.

    Matching normalized intrinsics reuse the original image, regardless of
    resolution. Invalid calibration raises ObservationError before any model
    request.
    """
    rgb, depth = observation.intrinsics, observation.depth_intrinsics
    if depth is None:
        raise ObservationError("model visual inputs require calibrated depth intrinsics")
    if observation.depth_image is None:
        raise ObservationError("model visual inputs require a depth visualisation image")
    for k in (rgb, depth):
        _validate_intrinsics(k)
    if (observation.rgb.shape[:2] != (rgb.height, rgb.width) or
            np.shape(observation.depth_image)[:2] != (depth.height, depth.width) or
            np.shape(observation.depth_m) != (depth.height, depth.width)):
        raise ObservationError("model images do not match their calibrated dimensions")
    # The CARLA rig renders both images from one camera, so normalized
    # intrinsics always match and the existing visualisation is reused.
    return observation.depth_image