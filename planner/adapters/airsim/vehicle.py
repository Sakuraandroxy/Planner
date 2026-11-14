from __future__ import annotations

import math
from planner.adapters.airsim.connection import AirSimConnection
from planner.domain.pose import WorldPose
from planner.domain.trajectory import MotionSegment
from planner.errors import ExecutionError
from planner.domain.motion import MotionLimits
from planner.adapters.airsim.motion_tracker import AirSimMotionTracker
from planner.services.motion.synchronized import SMOOTH_PROFILE

class AirSimVehicle:
    def __init__(self, connection: AirSimConnection, speed_mps: float, timeout_s: float,
                 motion_limits: MotionLimits | None = None):
        self.connection = connection
        self.speed_mps = speed_mps
        self.timeout_s = timeout_s
        self.motion_tracker = AirSimMotionTracker(
            connection, self.current_pose, motion_limits or MotionLimits(), timeout_s,
        )

    def current_pose(self) -> WorldPose:
        pose = self.connection.call("simGetVehiclePose")
        q = pose.orientation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
            1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val),
        ))
        return WorldPose(pose.position.x_val, pose.position.y_val, pose.position.z_val, yaw)

    def execute_segment(self, segment: MotionSegment) -> None:
        if segment.profile != SMOOTH_PROFILE:
            raise ExecutionError(f"unsupported motion profile: {segment.profile}")
        self.motion_tracker.execute(segment)

    def cancel(self) -> None:
        self.connection.call("cancelLastTask")

    def hover(self) -> None:
        self.connection.call_async_and_wait("hoverAsync")

    def collision_state(self) -> bool:
        return bool(self.connection.call("simGetCollisionInfo").has_collided)

