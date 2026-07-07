"""3DG-VLN 方向估计器——直接包含全部方向估计逻辑。"""
import math
from typing import List, Optional, Tuple

from agent.direction.base import BaseDirectionEstimator


# ─── 方向映射表 ───
LOCAL_DIRECTION_MAP = {
    "front-left": "It's off to the front-left",
    "straight": "It's straight ahead",
    "front-right": "It's off to the front-right",
    "down": "It's down below you",
}

DIRECTION_KEYS = ["front-left", "straight", "front-right", "down"]


class CameraModel:
    """从 3DG-VLN 源码移植的相机模型——像素→射线→yaw。"""

    def __init__(self, image_size, fov_deg=90.0):
        self.width, self.height = image_size
        self.fov = math.radians(fov_deg)
        self.fx = self.width / (2.0 * math.tan(self.fov / 2.0))
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

    def pixel_to_ray_yaw(self, px, py):
        """像素坐标 → 相机射线 → 偏航角（度）"""
        dx = (px - self.cx) / self.fx
        yaw_rad = math.atan2(dx, 1.0)
        return math.degrees(yaw_rad)


from agent.direction import register_direction


@register_direction("three_dg")
class ThreeDGDirectionEstimator(BaseDirectionEstimator):
    """3DG-VLN 方向估计器：下视检测优先，前视用像素→射线→方向分类。"""

    def estimate(self, bbox, camera_id, image_size) -> str:
        if bbox is None or len(bbox) < 4:
            return ""

        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        # 下视检测 → down
        if camera_id == 1:
            return LOCAL_DIRECTION_MAP["down"]

        # 前视检测 → 像素→射线→分类
        cam = CameraModel(image_size, fov_deg=90.0)
        yaw_deg = cam.pixel_to_ray_yaw(cx, cy)

        if yaw_deg < -15:
            return LOCAL_DIRECTION_MAP["front-left"]
        elif yaw_deg > 15:
            return LOCAL_DIRECTION_MAP["front-right"]
        else:
            return LOCAL_DIRECTION_MAP["straight"]


# ─── 兼容旧接口的函数 ───
def estimate_direction(bbox, camera_id=0, image_size=(256, 256)):
    est = ThreeDGDirectionEstimator()
    return est.estimate(bbox, camera_id, image_size)


def direction_text_to_key(text):
    for key, val in LOCAL_DIRECTION_MAP.items():
        if val == text:
            return key
    return "straight"
