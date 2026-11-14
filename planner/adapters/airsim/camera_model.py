from __future__ import annotations

import math

from planner.domain.observation import CameraIntrinsics

#根据图像尺寸 + 水平 FOV，算相机内参
def intrinsics_from_horizontal_fov(width: int, height: int, horizontal_fov_deg: float) -> CameraIntrinsics:
    width = max(1, int(width))
    height = max(1, int(height))
    fov = max(0.001, min(179.0, float(horizontal_fov_deg)))
    focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    return CameraIntrinsics(width, height, focal, focal, width / 2.0, height / 2.0, fov)

#把四元数姿态转换成 3×3 旋转矩阵
def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> tuple[tuple[float, float, float], ...]:
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )

