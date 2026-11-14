"""Synchronized observations from a ProjectAirSim camera sensor.

Stopped capture waits for a pair captured after the call began, stepping the
paused clock in small increments because ProjectAirSim only renders while time
advances. Moving capture reads the newest published pair without commanding the
world; the control loop owns stepping. In both cases the observation pose is
recovered from the camera pose published with the frame (the reference AirSim
convention), not from a separately sampled kinematics response.
"""

from __future__ import annotations

import time
import uuid

import cv2
import numpy as np

from ...config import CameraSettings, DepthSettings
from ...domain.depth_visualization import render_depth
from ...domain.execution import ExecutionBudget
from ...domain.observation import CameraIntrinsics, Observation
from ...errors import BudgetExceeded, ObservationError
from .camera import assert_camera_matches_scene, intrinsics_from_camera, sensor_configuration
from .connection import ProjectAirSimConnection
from .coordinates import (
    body_pose_from_camera,
    camera_matrix,
    camera_position_from_message,
    rotation_from_camera_message,
    rpy_rotation_deg,
)
from .frames import CameraTopicStream, FramePair


def _camera_topics(robot, camera_id: str) -> tuple[str, str]:
    """The server-published scene/depth topics for ``camera_id``.

    Names follow projectairsim's ``Robot.set_sensor_topics`` convention. The
    shipped Rover client overrides ``set_topics`` without building its sensor
    map, so derive them from the robot's parent topic instead of
    ``robot.sensors``.
    """
    parent_topic = getattr(robot, "parent_topic", "")
    if not parent_topic:
        raise ValueError(
            "robot has no parent topic; create it through projectairsim's "
            "Rover/Drone client"
        )
    root = f"{parent_topic}/sensors/{camera_id}"
    return f"{root}/scene_camera", f"{root}/depth_camera"


class ProjectAirSimObservationSource:
    def __init__(
        self,
        connection: ProjectAirSimConnection,
        robot,
        *,
        camera: CameraSettings,
        depth: DepthSettings,
        camera_id: str,
        frame_sink=None,
    ):
        self._connection = connection
        self._camera_id = str(camera_id)
        self._depth_settings = depth
        self.intrinsics = intrinsics_from_camera(
            camera.width, camera.height, camera.hfov_deg
        )
        self._mount_position = tuple(float(value) for value in camera.offset_m)
        self._mount_rotation = rpy_rotation_deg(camera.mount_rpy_deg)
        sensor = sensor_configuration(
            connection.scene_configuration(), robot.name, self._camera_id
        )
        assert_camera_matches_scene(
            sensor,
            width=camera.width,
            height=camera.height,
            hfov_deg=camera.hfov_deg,
            offset_m=self._mount_position,
            mount_rpy_deg=camera.mount_rpy_deg,
        )
        rgb_topic, depth_topic = _camera_topics(robot, self._camera_id)
        self._stream = CameraTopicStream(
            connection.client,
            rgb_topic=rgb_topic,
            depth_topic=depth_topic,
            width=camera.width,
            height=camera.height,
            frame_sink=frame_sink,
        )
        self._stream.start()
        self._sequence = 0

    # -- capture -----------------------------------------------------------
    def capture(
        self,
        *,
        budget: ExecutionBudget | None = None,
        timeout_s: float = 2.0,
        step_s: float = 0.05,
    ) -> Observation:
        """Wait for a fresh synchronized pair while stepping the paused clock."""
        started = time.monotonic()
        if budget is not None:
            remaining = budget.remaining(started)
            if remaining <= 0:
                raise BudgetExceeded("deadline before stopped observation")
            timeout_s = min(float(timeout_s), remaining)
        after_ns = self._connection.sim_time_ns()
        while True:
            pair = self._stream.poll_next(after_ns)
            if pair is not None:
                return self._observation(pair, stopped=True)
            if budget is not None and budget.remaining(time.monotonic()) <= 0:
                raise BudgetExceeded("deadline while waiting for a camera frame")
            self._connection.advance_for_sim_time(step_s)
            if time.monotonic() - started >= timeout_s:
                raise ObservationError(
                    "no synchronized RGB/depth frame arrived after the capture request"
                )

    def capture_moving(self, *, budget: ExecutionBudget | None = None) -> Observation:
        """Read the newest published pair; the control loop advances the clock."""
        if budget is not None and budget.remaining(time.monotonic()) <= 0:
            raise BudgetExceeded("deadline before moving observation")
        pair = self._stream.latest_pair()
        if pair is None:
            raise ObservationError(
                "no synchronized RGB/depth frame is available yet"
            )
        return self._observation(pair, stopped=False)

    def close(self) -> None:
        self._stream.close()

    # -- decoding ----------------------------------------------------------
    def _observation(self, pair: FramePair, *, stopped: bool) -> Observation:
        rgb_message = pair.rgb_message
        depth_message = pair.depth_message
        self._validate_registered_pair(rgb_message, depth_message)
        bgr = pair.bgr if pair.bgr is not None else self._stream.decode_scene(rgb_message)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        depth_m = self._stream.decode_depth(depth_message)
        rotation = rotation_from_camera_message(rgb_message)
        camera_position = camera_position_from_message(rgb_message)
        pose = body_pose_from_camera(
            camera_position, rotation, self._mount_position, self._mount_rotation
        )
        matrix = camera_matrix(camera_position, rotation)
        self._sequence += 1
        return Observation(
            frame=self._sequence,
            timestamp_s=pair.timestamp_ns / 1e9,
            pose=pose,
            rgb=rgb,
            depth_m=depth_m,
            intrinsics=self.intrinsics,
            camera_matrix=matrix,
            camera_id=self._camera_id,
            timestamp_ns=pair.timestamp_ns,
            captured_monotonic=time.monotonic(),
            capture_id=uuid.uuid4().hex,
            stopped=stopped,
            depth_image=render_depth(
                depth_m,
                self._depth_settings.min_m,
                self._depth_settings.max_m,
                self._depth_settings.mapping,
            ),
            depth_kind="ray_range",
            depth_timestamp_ns=pair.timestamp_ns,
            camera_position_world=camera_position,
            rotation_camera_to_world=rotation,
            depth_intrinsics=self.intrinsics,
            depth_mapping=self._depth_settings.mapping,
            depth_min_m=self._depth_settings.min_m,
            depth_max_m=self._depth_settings.max_m,
        )

    @staticmethod
    def _validate_registered_pair(rgb_message: dict, depth_message: dict) -> None:
        # One camera registration is a precondition of the ray mapping: both
        # image types must report the same camera pose for the same capture.
        rgb_position = np.array(camera_position_from_message(rgb_message))
        depth_position = np.array(camera_position_from_message(depth_message))
        rgb_rotation = rotation_from_camera_message(rgb_message)
        depth_rotation = rotation_from_camera_message(depth_message)
        if not np.allclose(rgb_position, depth_position, atol=1e-3, rtol=0) or not np.allclose(
            rgb_rotation, depth_rotation, atol=1e-3, rtol=0
        ):
            raise ObservationError(
                "scene and depth topic camera poses are not registered"
            )