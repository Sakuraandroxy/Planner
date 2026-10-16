from __future__ import annotations

import math
from planner.adapters.airsim.connection import AirSimConnection
from planner.domain.pose import WorldPose
from planner.domain.execution import ExecutionBudget, PoseCheckpoint
from planner.domain.trajectory import MotionSegment
from planner.errors import ExecutionError
from planner.domain.motion import MotionLimits
from planner.adapters.airsim.motion_tracker import AirSimMotionTracker
from planner.services.motion.synchronized import SMOOTH_PROFILE
from planner.domain.streaming import TimedVehicleState
import time

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

    def execute_segment(self, segment: MotionSegment, *, budget: ExecutionBudget | None = None,
                        pose_checkpoint: PoseCheckpoint | None = None) -> None:
        if segment.profile != SMOOTH_PROFILE:
            raise ExecutionError(f"unsupported motion profile: {segment.profile}")
        if pose_checkpoint is not None:
            self.motion_tracker.execute(segment, budget=budget, pose_checkpoint=pose_checkpoint)
        elif budget is None:
            self.motion_tracker.execute(segment)
        else:
            self.motion_tracker.execute(segment, budget=budget)

    def cancel(self) -> None:
        self.connection.call("cancelLastTask")

    def execute_basic_motion(self, start: WorldPose, target: WorldPose, yaw_delta_deg: float) -> None:
        """Send native commands; do not run trajectory, collision or arrival checks."""
        import airsim

        delta = (target.x-start.x, target.y-start.y, target.z-start.z)
        distance = math.hypot(*delta)
        if yaw_delta_deg:
            duration = max(distance/self.speed_mps,
                           abs(yaw_delta_deg)/self.motion_tracker.limits.max_yaw_rate_deg_s)
            rate = yaw_delta_deg/duration
            if distance:
                self.connection.call_async_and_wait(
                    "moveByVelocityAsync", *(v/duration for v in delta), duration,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=rate))
            else:
                self.connection.call_async_and_wait("rotateByYawRateAsync", rate, duration)
        else:
            self.connection.call_async_and_wait(
                "moveToPositionAsync", target.x, target.y, target.z, self.speed_mps,
                timeout_sec=self.timeout_s, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0))
        self.hover()

    def hover(self) -> None:
        self.connection.call_async_and_wait("hoverAsync")

    def collision_state(self) -> bool:
        return bool(self.connection.call("simGetCollisionInfo").has_collided)

    def stream_state(self) -> TimedVehicleState:
        state = self.connection.call("getMultirotorState")
        k = state.kinematics_estimated
        q, p, v = k.orientation, k.position, k.linear_velocity
        yaw = math.degrees(math.atan2(2*(q.w_val*q.z_val+q.x_val*q.y_val),
                                     1-2*(q.y_val*q.y_val+q.z_val*q.z_val)))
        return TimedVehicleState(WorldPose(p.x_val, p.y_val, p.z_val, yaw),
            int(state.timestamp), time.monotonic(), (v.x_val, v.y_val, v.z_val))

    def stream_velocity(self, velocity_world, yaw_rate_deg_s, duration_s):
        import airsim
        if (len(velocity_world) != 3 or not all(math.isfinite(v) for v in
                (*velocity_world, yaw_rate_deg_s, duration_s)) or duration_s <= 0):
            raise ExecutionError("non-finite streaming velocity or invalid duration")
        self.connection.call_async_and_wait("moveByVelocityAsync", *velocity_world, duration_s,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg_s))

