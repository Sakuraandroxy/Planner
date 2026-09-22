from __future__ import annotations

import math
import logging

from planner.adapters.airsim.connection import AirSimConnection
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import MotionSegment
from planner.errors import ExecutionError
from planner.domain.motion import MotionLimits
from planner.adapters.airsim.motion_tracker import AirSimMotionTracker
from planner.services.motion.synchronized import SMOOTH_PROFILE

logger = logging.getLogger(__name__)
ROTATION_YAW_TOLERANCE_DEG = 2.0


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
        if segment.profile == SMOOTH_PROFILE:
            self.motion_tracker.execute(segment)
            return
        if segment.profile != "linear":
            raise ExecutionError(f"unsupported motion profile: {segment.profile}")
        end = segment.end
        translation = math.dist(
            (segment.start.x, segment.start.y, segment.start.z),
            (end.x, end.y, end.z),
        )
        if translation <= 1e-6:
            self._rotate_to_yaw(end.yaw_deg)
            return

        import airsim

        speed = segment.target_speed_mps or self.speed_mps
        self.connection.call_async_and_wait(
            "moveToPositionAsync",
            end.x,
            end.y,
            end.z,
            speed,
            timeout_sec=self.timeout_s,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=end.yaw_deg),
        )

    def _rotate_to_yaw(self, target_yaw_deg: float) -> None:
        target = wrap_yaw_deg(target_yaw_deg)
        current = self.current_pose()
        if abs(wrap_yaw_deg(current.yaw_deg - target)) <= ROTATION_YAW_TOLERANCE_DEG:
            logger.info("[Vehicle] yaw already within tolerance: actual=%.2f target=%.2f", current.yaw_deg, target)
            return
        logger.info("[Vehicle] rotate in place: current=%.2f target=%.2f", current.yaw_deg, target)
        self.connection.call_async_and_wait(
            "rotateToYawAsync", target,
            timeout_sec=self.timeout_s, margin=ROTATION_YAW_TOLERANCE_DEG,
        )
        actual = self.current_pose().yaw_deg
        error = abs(wrap_yaw_deg(actual - target))
        logger.info("[Vehicle] rotation result: actual=%.2f target=%.2f error=%.2f deg", actual, target, error)
        if error > ROTATION_YAW_TOLERANCE_DEG:
            raise ExecutionError(
                f"rotation did not reach target: target={target:.2f}, actual={actual:.2f}, error={error:.2f} deg"
            )

    def cancel(self) -> None:
        self.connection.call("cancelLastTask")

    def hover(self) -> None:
        self.connection.call_async_and_wait("hoverAsync")

    def collision_state(self) -> bool:
        return bool(self.connection.call("simGetCollisionInfo").has_collided)

