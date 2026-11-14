from __future__ import annotations

import math
import numpy as np
from numbers import Integral

from planner.domain.observation import CameraIntrinsics

#根据图像尺寸 + 水平 FOV，算相机内参
def intrinsics_from_horizontal_fov(width: int, height: int, horizontal_fov_deg: float) -> CameraIntrinsics:
    if any(isinstance(value, bool) or not isinstance(value, Integral) or value <= 0 for value in (width, height)):
        raise ValueError("camera image dimensions must be positive integers")
    try:
        fov = float(horizontal_fov_deg)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("camera horizontal FOV must be numeric") from exc
    if isinstance(horizontal_fov_deg, bool) or not math.isfinite(fov) or not 0 < fov < 180:
        raise ValueError("camera horizontal FOV must be finite and between 0 and 180 degrees")
    focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    return CameraIntrinsics(width, height, focal, focal, width / 2.0, height / 2.0, fov)

#把四元数姿态转换成 3×3 旋转矩阵
def camera_matrix(
    camera_position: tuple[float, float, float], camera_rotation
) -> "np.ndarray":
    """4x4 NED camera-to-world transform (rotation + translation).

    ``Observation.__post_init__`` fills ``camera_position_world`` and
    ``rotation_camera_to_world`` from this matrix, so the layout is part of the
    shared contract rather than a local choice. It is built here because the UE4
    runtime cannot import the sibling simulator package to borrow a copy.
    """
    rotation = np.asarray(camera_rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("camera rotation must be 3x3")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(camera_position, dtype=np.float64)
    return matrix


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> tuple[tuple[float, float, float], ...]:
    try:
        finite = all(not isinstance(value, bool) and math.isfinite(value) for value in (x, y, z, w))
    except (TypeError, ValueError, OverflowError):
        finite = False
    if not finite:
        raise ValueError("camera quaternion must be finite")
    norm = math.hypot(x, y, z, w)
    if not math.isclose(norm, 1.0, abs_tol=1e-3, rel_tol=0):
        raise ValueError("camera quaternion must have unit norm")
    # Normalize only small floating-point deviations of valid unit quaternions.
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )

