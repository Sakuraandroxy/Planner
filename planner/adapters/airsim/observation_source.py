from __future__ import annotations

import math
import uuid
import time
from contextlib import nullcontext

import numpy as np
from PIL import Image

from planner.adapters.airsim.camera_model import intrinsics_from_horizontal_fov, quaternion_to_rotation
from planner.adapters.airsim.connection import AirSimConnection
from planner.domain.observation import Observation
from planner.ports.vehicle import Vehicle
from planner.errors import BudgetExceeded, ObservationError
from .depth_visualization import render_depth


class AirSimObservationSource:
    def __init__(
        self,
        connection: AirSimConnection,
        vehicle: Vehicle | None,
        camera_id: str,
        min_depth_m: float,
        max_depth_m: float,
        observation_gate=None,
        depth_horizontal_fov_deg: float | None = None,
        verify_image_pair: bool = False,
        depth_mapping: str = "log_inverse_gray_v1",
        maximum_pair_time_gap_s: float = 0.05,
        camera_mount_position_body_m=(1.0, 0.0, 0.0),
        camera_mount_rpy_deg=(0.0, 0.0, 0.0),
    ):
        self.connection = connection
        self.vehicle = vehicle
        self.camera_id = camera_id
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self._horizontal_fov_deg: float | None = None
        self.observation_gate = observation_gate
        self.depth_horizontal_fov_deg = depth_horizontal_fov_deg
        self.verify_image_pair = verify_image_pair
        self.depth_mapping = depth_mapping
        self.maximum_pair_time_gap_s = maximum_pair_time_gap_s
        self.camera_mount_position_body_m = camera_mount_position_body_m
        roll, pitch, yaw = (math.radians(v)/2 for v in camera_mount_rpy_deg)
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        self.mount_rotation = np.asarray(quaternion_to_rotation(
            sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy))

    def capture(self, *, budget=None) -> Observation:
        try:
            if self.vehicle is None:
                raise ObservationError("read-only moving source cannot take stopped observations")
            if budget is not None and budget.remaining(time.monotonic()) <= 0:
                raise BudgetExceeded("deadline before observation")
            guard = (self.observation_gate.capture(budget=budget) if self.observation_gate else nullcontext())
            with guard:
                result = self._capture()
                if budget is not None and budget.remaining(time.monotonic()) <= 0:
                    raise BudgetExceeded("late observation discarded")
                return result
        except BudgetExceeded:
            raise
        except Exception as exc:
            raise ObservationError(str(exc)) from exc

    def request_capture(self, timeout_s=None):
        if self.observation_gate is None:
            raise ObservationError("formal observation requires a control-owned gate")
        return self.observation_gate.request_capture(self._capture, timeout_s)

    def capture_moving(self, *, budget=None) -> Observation:
        if budget is not None and budget.remaining(time.monotonic()) <= 0:
            raise BudgetExceeded("deadline before moving observation")
        return self._capture(moving=True)

    def close(self):
        self.connection.close_transport()

    def _capture(self, *, moving=False) -> Observation:
        import airsim

        capture_started = time.monotonic()
        responses = self.connection.call("simGetImages", [
            airsim.ImageRequest(self.camera_id, airsim.ImageType.Scene, False, True),
            airsim.ImageRequest(self.camera_id, airsim.ImageType.DepthPerspective, True, False),
        ])
        if len(responses) != 2 or not responses[0].image_data_uint8 or not responses[1].image_data_float:
            raise RuntimeError(f"camera {self.camera_id!r} returned an incomplete RGB/depth pair")
        rgb_response, depth_response = responses
        captured = time.monotonic()
        if self.verify_image_pair:
            self._validate_pair(rgb_response, depth_response, self.maximum_pair_time_gap_s)
        vehicle_pose = None if moving else self.vehicle.current_pose()
        rgb = _decode_rgb(rgb_response.image_data_uint8)
        depth = np.asarray(depth_response.image_data_float, dtype=np.float32).reshape(
            depth_response.height, depth_response.width
        )
        camera_position = rgb_response.camera_position
        camera_orientation = rgb_response.camera_orientation
        q = camera_orientation
        rotation = quaternion_to_rotation(q.x_val, q.y_val, q.z_val, q.w_val)
        if moving:
            from planner.domain.pose import WorldPose
            body_rotation = np.asarray(rotation) @ self.mount_rotation.T
            body_position = np.array((camera_position.x_val, camera_position.y_val, camera_position.z_val))-body_rotation @ np.asarray(self.camera_mount_position_body_m)
            vehicle_pose = WorldPose(*map(float, body_position), math.degrees(math.atan2(body_rotation[1, 0], body_rotation[0, 0])))
        intrinsics = intrinsics_from_horizontal_fov(
            rgb.width, rgb.height, self._get_horizontal_fov_deg()
        )
        timestamp_ns = int(getattr(rgb_response, "time_stamp", 0) or 0)
        return Observation(
            rgb=rgb,
            depth_meters=depth,
            depth_image=render_depth(depth, self.min_depth_m, self.max_depth_m, self.depth_mapping),
            depth_mapping=self.depth_mapping, depth_min_m=self.min_depth_m, depth_max_m=self.max_depth_m,
            vehicle_pose=vehicle_pose,
            camera_id=self.camera_id,
            intrinsics=intrinsics,
            camera_position_world=(camera_position.x_val, camera_position.y_val, camera_position.z_val),
            rotation_camera_to_world=rotation,
            timestamp_ns=timestamp_ns,
            depth_intrinsics=intrinsics_from_horizontal_fov(
                depth_response.width, depth_response.height,
                self.depth_horizontal_fov_deg or self._get_horizontal_fov_deg(),
            ),
            depth_kind="ray_range",
            captured_monotonic=capture_started if moving else captured,
            capture_id=uuid.uuid4().hex,
            depth_timestamp_ns=int(getattr(depth_response, "time_stamp", 0) or 0),
            stopped=not moving and self.observation_gate is not None,
            capture_started_monotonic=capture_started,
            capture_finished_monotonic=time.monotonic(),
        )

    @staticmethod
    def _validate_pair(rgb_response, depth_response, maximum_pair_time_gap_s=0.05):
        times = [getattr(r, "time_stamp", 0) for r in (rgb_response, depth_response)]
        if (any(type(t) is not int or t <= 0 for t in times) or
                abs(times[0]-times[1]) > maximum_pair_time_gap_s*1e9):
            raise ObservationError("RGB/depth timestamps are missing or exceed the configured pair gap")
        # One camera registration is a precondition of the ray mapping. Image
        # response poses must agree; FOV/crop calibration is verified separately.
        positions, rotations = [], []
        for response in (rgb_response, depth_response):
            p, q = response.camera_position, response.camera_orientation
            positions.append((p.x_val, p.y_val, p.z_val))
            rotations.append(quaternion_to_rotation(q.x_val, q.y_val, q.z_val, q.w_val))
        if not np.allclose(positions[0], positions[1], atol=1e-3, rtol=0) or not np.allclose(
            rotations[0], rotations[1], atol=1e-3, rtol=0
        ):
            raise ObservationError("RGB/depth response camera poses are not registered")

    def _get_horizontal_fov_deg(self) -> float:
        if self._horizontal_fov_deg is None:
            camera_info = self.connection.call("simGetCameraInfo", self.camera_id)
            self._horizontal_fov_deg = float(camera_info.fov)
        return self._horizontal_fov_deg


def _decode_rgb(data: bytes) -> Image.Image:
    import io

    return Image.open(io.BytesIO(data)).convert("RGB")


def _depth_visualization(depth: np.ndarray, minimum: float, maximum: float) -> Image.Image:
    """Compatibility entry point for the legacy renderer."""
    return render_depth(depth, minimum, maximum)
