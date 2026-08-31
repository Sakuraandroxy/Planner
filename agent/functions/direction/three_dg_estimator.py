"""3DG-VLN-style coarse direction estimator."""

from __future__ import annotations

import math

from agent.functions.direction import register_direction
from agent.functions.direction.base import BaseDirectionEstimator

LOCAL_DIRECTION_MAP = {
    "front-left": "It's off to the front-left",
    "straight": "It's straight ahead",
    "front-right": "It's off to the front-right",
    "down": "It's down below you",
}
DIRECTION_KEYS = ["front-left", "straight", "front-right", "down"]


class CameraModel:
    """Simple pinhole camera model for pixel-to-yaw mapping."""

    def __init__(self, image_size, fov_deg=90.0):
        self.width, self.height = image_size
        self.fov = math.radians(fov_deg)
        self.fx = self.width / (2.0 * math.tan(self.fov / 2.0))
        self.cx = self.width / 2.0

    def pixel_to_ray_yaw(self, px, py):
        dx = (px - self.cx) / self.fx
        return math.degrees(math.atan2(dx, 1.0))


@register_direction("three_dg")
class ThreeDGDirectionEstimator(BaseDirectionEstimator):
    """Classify a detection as front-left, straight, front-right, or down."""

    def estimate(
        self,
        bbox=None,
        camera_id=0,
        image_size=(256, 256),
        *,
        detection=None,
        navigation_yaw_deg: float = 0.0,
    ) -> str:
        """Classify by a world ray when available, with a legacy adapter.

        ``camera_id == 1`` is intentionally retained only in the no-frame
        adapter so old callers keep working while core geometry remains
        independent of camera names and configured pitch.
        """
        detection = detection or (bbox if hasattr(bbox, "world_ray") else None)
        if detection is not None:
            bbox = getattr(detection, "bbox", bbox)
            world_ray = getattr(detection, "world_ray", None)
            if world_ray is not None:
                direction = world_ray.direction_world
                horizontal = math.hypot(float(direction[0]), float(direction[1]))
                if horizontal <= 1e-9:
                    return LOCAL_DIRECTION_MAP["down"] if float(direction[2]) > 0.0 else "Target is above the drone."
                world_bearing = math.degrees(math.atan2(float(direction[1]), float(direction[0])))
                yaw_deg = (world_bearing - float(navigation_yaw_deg) + 180.0) % 360.0 - 180.0
                if yaw_deg < -15:
                    return LOCAL_DIRECTION_MAP["front-left"]
                if yaw_deg > 15:
                    return LOCAL_DIRECTION_MAP["front-right"]
                return LOCAL_DIRECTION_MAP["straight"]
        if bbox is None or len(bbox) < 4:
            return ""

        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        if camera_id == 1:
            return LOCAL_DIRECTION_MAP["down"]

        yaw_deg = CameraModel(image_size, fov_deg=90.0).pixel_to_ray_yaw(cx, cy)
        if yaw_deg < -15:
            return LOCAL_DIRECTION_MAP["front-left"]
        if yaw_deg > 15:
            return LOCAL_DIRECTION_MAP["front-right"]
        return LOCAL_DIRECTION_MAP["straight"]


def estimate_direction(bbox, camera_id=0, image_size=(256, 256)):
    return ThreeDGDirectionEstimator().estimate(bbox, camera_id, image_size)


def direction_text_to_key(text):
    for key, val in LOCAL_DIRECTION_MAP.items():
        if val == text:
            return key
    return "straight"
