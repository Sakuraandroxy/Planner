from __future__ import annotations

import math
import uuid

import numpy as np
from PIL import Image

from planner.adapters.airsim.camera_model import intrinsics_from_horizontal_fov, quaternion_to_rotation
from planner.adapters.airsim.connection import AirSimConnection
from planner.domain.observation import Observation
from planner.ports.vehicle import Vehicle


class AirSimObservationSource:
    def __init__(
        self,
        connection: AirSimConnection,
        vehicle: Vehicle,
        camera_id: str,
        min_depth_m: float,
        max_depth_m: float,
    ):
        self.connection = connection
        self.vehicle = vehicle
        self.camera_id = camera_id
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m

    def capture(self) -> Observation:
        import airsim

        responses = self.connection.call("simGetImages", [
            airsim.ImageRequest(self.camera_id, airsim.ImageType.Scene, False, True),
            airsim.ImageRequest(self.camera_id, airsim.ImageType.DepthPerspective, True, False),
        ])
        if len(responses) != 2 or not responses[0].image_data_uint8 or not responses[1].image_data_float:
            raise RuntimeError(f"camera {self.camera_id!r} returned an incomplete RGB/depth pair")
        rgb_response, depth_response = responses
        rgb = _decode_rgb(rgb_response.image_data_uint8)
        depth = np.asarray(depth_response.image_data_float, dtype=np.float32).reshape(
            depth_response.height, depth_response.width
        )
        camera_info = self.connection.call("simGetCameraInfo", self.camera_id)
        pose = camera_info.pose
        q = pose.orientation
        rotation = quaternion_to_rotation(q.x_val, q.y_val, q.z_val, q.w_val)
        intrinsics = intrinsics_from_horizontal_fov(rgb.width, rgb.height, camera_info.fov)
        timestamp_ns = int(getattr(rgb_response, "time_stamp", 0) or 0)
        return Observation(
            rgb=rgb,
            depth_meters=depth,
            depth_image=_depth_visualization(depth, self.min_depth_m, self.max_depth_m),
            vehicle_pose=self.vehicle.current_pose(),
            camera_id=self.camera_id,
            intrinsics=intrinsics,
            camera_position_world=(pose.position.x_val, pose.position.y_val, pose.position.z_val),
            rotation_camera_to_world=rotation,
            timestamp_ns=timestamp_ns,
        )


def _decode_rgb(data: bytes) -> Image.Image:
    import io

    return Image.open(io.BytesIO(data)).convert("RGB")


def _depth_visualization(depth: np.ndarray, minimum: float, maximum: float) -> Image.Image:
    valid = np.isfinite(depth) & (depth >= minimum) & (depth <= maximum)
    output = np.zeros(depth.shape, dtype=np.uint8)
    if np.any(valid):
        denominator = math.log(maximum) - math.log(minimum)
        values = (math.log(maximum) - np.log(np.clip(depth[valid], minimum, maximum))) / denominator
        output[valid] = np.clip(values * 255.0, 1, 255).astype(np.uint8)
    return Image.fromarray(output, mode="L").convert("RGB")

