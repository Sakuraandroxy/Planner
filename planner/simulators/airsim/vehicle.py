from __future__ import annotations

import math
import logging
from planner.simulators.airsim.connection import AirSimConnection
from planner.domain.pose import WorldPose
from planner.domain.execution import ExecutionBudget, PoseCheckpoint
from planner.domain.trajectory import MotionSegment
from planner.errors import ExecutionError
from planner.domain.motion import MotionLimits
from planner.simulators.airsim.motion_tracker import AirSimMotionTracker
from planner.services.motion.synchronized import SMOOTH_PROFILE
from planner.domain.streaming import TimedVehicleState
import time

logger = logging.getLogger(__name__)

#: How close the aircraft has to be to the commanded endpoint for the motion to
#: count as executed. Measured on Blocks: a commanded climb settles within 6 cm.
ARRIVAL_TOLERANCE_M = 0.5
#: Yaw is verified from state for the same reason position is. Loose, because the
#: endpoint of a signed rotation is the thing the caller asked for, not a heading.
ARRIVAL_TOLERANCE_DEG = 15.0


class AirSimVehicle:
    def __init__(self, connection: AirSimConnection, speed_mps: float, timeout_s: float,
                 motion_limits: MotionLimits | None = None, *, velocity_command_duration_s: float = .2):
        self.connection = connection
        self.speed_mps = speed_mps
        self.timeout_s = timeout_s
        self.holding = None
        self.velocity_command_duration_s = velocity_command_duration_s
        self.motion_tracker = AirSimMotionTracker(
            connection, self.current_pose, motion_limits or MotionLimits(), timeout_s,
        )

    @property
    def tracking_limits(self) -> MotionLimits:
        """The envelope the mission assembly plans against.

        ProjectAirSim converts its own motion config into domain units; AirSim is
        configured in domain units already, so this is the object the vehicle was
        built with. The property exists so the shared assembly has one attribute to
        ask every vehicle for instead of branching per simulator.
        """
        return self.motion_tracker.limits

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
        if self.holding is not None:
            self.holding.release()
        if segment.profile != SMOOTH_PROFILE:
            raise ExecutionError(f"unsupported motion profile: {segment.profile}")
        if pose_checkpoint is not None:
            self.motion_tracker.execute(segment, budget=budget, pose_checkpoint=pose_checkpoint)
        elif budget is None:
            self.motion_tracker.execute(segment)
        else:
            self.motion_tracker.execute(segment, budget=budget)
        if self.holding is not None:
            self.holding.start(segment.end)
            self.holding.wait(budget=budget)

    def cancel(self) -> None:
        self.connection.call("cancelLastTask")

    def _await_arrival(self, target: WorldPose, *, yaw_delta_deg: float = 0.0) -> None:
        """Decide from state that the commanded endpoint was reached.

        This build does not report motion truthfully: measured on the machine,
        ``takeoffAsync().get()`` returned ``False`` while the aircraft climbed from
        z=0.68 to z=-0.86, and ``moveToPositionAsync().get()`` returned ``False`` a
        metre up. ``landed_state`` is not a substitute either -- after a confirmed
        ``landAsync()`` that returned ``True`` it stayed at ``1`` (Flying) with the
        vehicle back on its spawn height. So the ack is logged and the position and
        heading are read back, which is the only evidence here that cannot lie.

        Raises ExecutionError when the vehicle has not settled inside the tolerances
        before ``timeout_s``, and says what it measured.
        """
        deadline = time.monotonic() + self.timeout_s
        pose = self.current_pose()
        while True:
            distance = math.sqrt(
                (pose.x - target.x) ** 2 + (pose.y - target.y) ** 2 + (pose.z - target.z) ** 2
            )
            yaw_error = abs(((pose.yaw_deg - target.yaw_deg + 180.0) % 360.0) - 180.0)
            if distance <= ARRIVAL_TOLERANCE_M and (
                not yaw_delta_deg or yaw_error <= ARRIVAL_TOLERANCE_DEG
            ):
                return
            if time.monotonic() >= deadline:
                raise ExecutionError(
                    f"AirSim basic motion did not reach its endpoint: {distance:.2f} m from "
                    f"{target} (tolerance {ARRIVAL_TOLERANCE_M} m), yaw error "
                    f"{yaw_error:.1f} deg after {self.timeout_s:g}s"
                )
            time.sleep(0.1)
            pose = self.current_pose()

    def execute_basic_motion(self, start: WorldPose, target: WorldPose, yaw_delta_deg: float) -> None:
        """Execute once toward a fixed endpoint; verify it through local hold."""
        import airsim

        native_target = None
        if self.holding is not None:
            self.holding.release()

        delta = (target.x-start.x, target.y-start.y, target.z-start.z)
        distance = math.hypot(*delta)
        if yaw_delta_deg:
            duration = max(distance/self.speed_mps,
                           abs(yaw_delta_deg)/self.motion_tracker.limits.max_yaw_rate_deg_s)
            rate = yaw_delta_deg/duration
            if distance:
                result = self.connection.call_async_and_wait(
                    "moveByVelocityAsync", *(v/duration for v in delta), duration,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=rate))
            else:
                result = self.connection.call_async_and_wait("rotateByYawRateAsync", rate, duration)
        else:
            native_target = self.holding.approach_target(start,target) if self.holding is not None else target
            if native_target is None:
                logger.info("[BasicMotion] short translation uses endpoint hold: target=%s",target)
                result = True
            else:
                logger.info("[BasicMotion] native translation handoff=%s final_target=%s",native_target,target)
                result = self.connection.call_async_and_wait(
                    "moveToPositionAsync", native_target.x, native_target.y, native_target.z, self.speed_mps,
                    timeout_sec=self.timeout_s, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0))
        if result is not True:
            logger.info("[BasicMotion] native ack=%r result_type=%s target=%s; deciding from state",
                        result,type(result).__name__,target)
        if self.holding is not None:
            # The hold service owns the final approach and its own waiting rules, but
            # the handoff point the native call was given still has to show up in
            # state -- an ack that says False used to be the only evidence here.
            if native_target is not None:
                self._await_arrival(native_target)
            self.holding.start(target)
            self.holding.wait()
            if yaw_delta_deg:
                self._await_arrival(target, yaw_delta_deg=yaw_delta_deg)
        else:
            self._await_arrival(target, yaw_delta_deg=yaw_delta_deg)
            self.hover()

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
        self.holding.wait_for_observation(budget=budget, checkpoint=checkpoint)

    def collision_state(self) -> bool:
        return bool(self.connection.call("simGetCollisionInfo").has_collided)

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
                                   1-2*(q.x_val*q.x_val+q.y_val*q.y_val))))

    def stream_velocity(self, velocity_world, yaw_rate_deg_s, duration_s):
        if self.holding is not None:
            self.holding.release()
        self.hold_velocity(velocity_world, yaw_rate_deg_s, duration_s)

    def hold_velocity(self, velocity_world, yaw_rate_deg_s, duration_s):
        import airsim
        if (len(velocity_world) != 3 or not all(math.isfinite(v) for v in
                (*velocity_world, yaw_rate_deg_s, duration_s)) or duration_s <= 0):
            raise ExecutionError("non-finite streaming velocity or invalid duration")
        self.connection.call_streaming("moveByVelocityAsync", *velocity_world,
            max(duration_s,self.velocity_command_duration_s),
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg_s))

