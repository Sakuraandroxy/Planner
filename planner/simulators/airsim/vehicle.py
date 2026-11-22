from __future__ import annotations

import logging
import math
import time

from planner.domain import ExecutionBudget, PoseCheckpoint
from planner.domain import CollisionState
from planner.domain import (
    MotionCapability,
    MotionCommand,
    MotionLimits,
    MotionModel,
    require_supported,
)
from planner.domain import WorldPose
from planner.domain import TimedVehicleState
from planner.domain import MotionSegment, MotionTrajectory
from planner.errors import ExecutionError
from planner.runtime import SMOOTH_PROFILE
from planner.simulators.airsim.connection import AirSimConnection

logger = logging.getLogger(__name__)


class AirSimVehicle:
    def __init__(self, connection: AirSimConnection, speed_mps: float, timeout_s: float,
                 motion_limits: MotionLimits | None = None, *, velocity_command_duration_s: float = .2):
        self.connection = connection
        self.speed_mps = speed_mps
        self.timeout_s = timeout_s
        self.holding = None
        self.velocity_command_duration_s = velocity_command_duration_s
        self._capability = MotionCapability(
            MotionModel.HOLONOMIC,
            max_lateral_speed_mps=(motion_limits or MotionLimits()).max_speed_mps,
            min_turn_radius_m=None,
            allows_reverse=True,
        )
        self._limits = motion_limits or MotionLimits()
        self._loop = None

    @property
    def tracking_limits(self) -> MotionLimits:
        """The envelope the mission assembly plans against.

        ProjectAirSim converts its own motion config into domain units; AirSim is
        configured in domain units already, so this is the object the vehicle was
        built with. The property exists so the shared assembly has one attribute to
        ask every vehicle for instead of branching per simulator.
        """
        return self._limits

    def current_pose(self) -> WorldPose:
        pose = self.connection.call("simGetVehiclePose")
        q = pose.orientation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
            1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val),
        ))
        return WorldPose(pose.position.x_val, pose.position.y_val, pose.position.z_val, yaw)

    def attach_loop(self, loop) -> None:
        """Transitional: assembly injects the shared control loop."""
        self._loop = loop

    def execute_segment(self, segment: MotionSegment, *, budget: ExecutionBudget | None = None,
                        pose_checkpoint: PoseCheckpoint | None = None) -> None:
        if self.holding is not None:
            self.holding.release()
        if segment.profile != SMOOTH_PROFILE:
            raise ExecutionError(f"unsupported motion profile: {segment.profile}")
        self._loop.run_segment(MotionTrajectory((segment,)), self._loop.segment_sink(),
                               budget=budget, pose_checkpoint=pose_checkpoint)
        if self.holding is not None:
            self.holding.start(segment.end)
            self.holding.wait(budget=budget)

    def cancel(self) -> None:
        self.connection.call("cancelLastTask")

    def hover(self) -> None:
        if self.holding is None:
            raise ExecutionError("AirSim position hold service was not assembled")
        self.holding.start()
        self.holding.tick()

    def hold_tick(self):
        if self.holding is not None:
            self.holding.tick()

    def begin_observation_hold(self):
        """Keep the existing fixed reference across consecutive image captures."""
        if self.holding is None or not self.holding.active:
            self.hover()
        else:
            self.holding.tick()

    def wait_for_observation(self, *, budget=None, checkpoint=None):
        if self.holding is None:
            raise ExecutionError("AirSim position hold service was not assembled")
        # The hold service exposes this as ``wait``: it ticks the hold until the state
        # is observable (settled across the whole window, not merely net displacement),
        # which is what a fresh image needs. The two names used to disagree and the
        # visual-goal stage died on "'PositionHold' object has no attribute
        # 'wait_for_observation'" -- measured on Blocks, one call after capture began
        # working. Reusing the hold's own wait is deliberate: a second stabilization
        # loop here would drift from the one motion already depends on.
        self.holding.wait(budget=budget, checkpoint=checkpoint)

    def motion_capability(self) -> MotionCapability:
        """The multirotor translates freely in the world frame."""
        return self._capability

    def collision_state(self) -> CollisionState:
        collided = bool(self.connection.call("simGetCollisionInfo").has_collided)
        return CollisionState(collided)

    def stream_state(self) -> TimedVehicleState:
        state = self.connection.call("getMultirotorState")
        k = state.kinematics_estimated
        q, p, v = k.orientation, k.position, k.linear_velocity
        yaw = math.degrees(math.atan2(2*(q.w_val*q.z_val+q.x_val*q.y_val),
                                     1-2*(q.y_val*q.y_val+q.z_val*q.z_val)))
        return TimedVehicleState(WorldPose(p.x_val, p.y_val, p.z_val, yaw),
            int(state.timestamp), time.monotonic(), (v.x_val, v.y_val, v.z_val),
            math.degrees(math.asin(max(-1., min(1., 2*(q.w_val*q.y_val-q.z_val*q.x_val))))),
            math.degrees(math.atan2(2*(q.w_val*q.x_val+q.y_val*q.z_val),
                                   1-2*(q.x_val*q.x_val+q.y_val*q.y_val))),
            yaw_rate_deg_s=math.degrees(k.angular_velocity.z_val))

    def stream_command(self, command: MotionCommand, duration_s):
        require_supported(command, self._capability)
        if self.holding is not None:
            self.holding.release()
        self.hold_velocity(
            (command.vx, command.vy, command.vz), command.yaw_rate_deg_s, duration_s
        )

    def hold_velocity(self, velocity_world, yaw_rate_deg_s, duration_s):
        import airsim
        if (len(velocity_world) != 3 or not all(math.isfinite(v) for v in
                (*velocity_world, yaw_rate_deg_s, duration_s)) or duration_s <= 0):
            raise ExecutionError("non-finite streaming velocity or invalid duration")
        self.connection.call_streaming("moveByVelocityAsync", *velocity_world,
            max(duration_s,self.velocity_command_duration_s),
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg_s))

