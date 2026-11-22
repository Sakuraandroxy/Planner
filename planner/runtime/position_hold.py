"""Owner-thread position holding shared by observation, task end and idle waits."""
import logging
import math
import threading
import time
from collections import deque

from planner.domain import ContinuousTrackingSettings, MotionLimits
from planner.domain import WorldPose, wrap_yaw_deg
from planner.domain import TimedVehicleState
from planner.errors import BudgetExceeded, ExecutionError
from planner.ports import PositionHoldVehicle
from planner.algorithms import bounded
from planner.runtime.motion.command_smoothing import CommandSmoother
from planner.runtime.motion.heading_servo import heading_rate_deg_s
from planner.runtime.motion.hold_handoff import translation_handoff

logger = logging.getLogger(__name__)


class PositionHold:
    def __init__(self, vehicle: PositionHoldVehicle, limits: MotionLimits,
                 tracking: ContinuousTrackingSettings, speed_mps: float, timeout_s: float,
                 *, clock=time.monotonic, sleep=time.sleep):
        self.vehicle, self.limits = vehicle, limits
        self.speed, self.timeout_s, self.clock, self.sleep = speed_mps, timeout_s, clock, sleep
        self.tracking = tracking
        self.translation = CommandSmoother(limits.max_acceleration_mps2, tracking.max_jerk_mps3)
        self.rotation = CommandSmoother(limits.max_yaw_acceleration_deg_s2, tracking.max_yaw_jerk_deg_s3)
        self.owner = threading.get_ident()
        self.target: WorldPose | None = None
        self.state: TimedVehicleState | None = None
        self.active = False
        self.samples = deque()
        self.last_tick = -math.inf
        self.last_diagnostic = -math.inf

    def _owner(self):
        if threading.get_ident() != self.owner:
            raise ExecutionError("position hold may only run on the control owner")

    def start(self, target: WorldPose | None = None) -> None:
        self._owner()
        state = self.vehicle.stream_state()
        self.target = state.pose if target is None else target
        self.vehicle.cancel()
        self.translation.reset(state.velocity_world)
        self.rotation.reset((0.,))
        self.samples.clear()
        self.state = state
        self.active = True
        self.last_tick = self.last_diagnostic = -math.inf
        logger.info("Hold reference NED: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                    self.target.x, self.target.y, self.target.z, self.target.yaw_deg)

    def release(self):
        self._owner()
        self.active = False
        self.samples.clear()

    def approach_target(self, start: WorldPose, target: WorldPose) -> WorldPose | None:
        """Return the native travel endpoint before this service brakes to target."""
        return translation_handoff(start,target,self.speed,self.limits.max_acceleration_mps2,
                                   self.translation.maximum_jerk)

    def tick(self):
        self._owner()
        now = self.clock()
        if not self.active or now-self.last_tick < 1/self.limits.control_hz:
            return
        state = self.vehicle.stream_state()
        previous = self.state
        dt = (state.timestamp_ns-previous.timestamp_ns)/1e9 if previous else 0
        self.state = state
        self.last_tick = now
        if dt < 0:
            raise ExecutionError("simulator clock moved backwards during position hold")
        frozen = dt <= 0
        if frozen:
            # A steppable simulator only advances while this service is
            # commanding it; the first tick after start() sees the unchanged
            # timestamp. Command one nominal period so the clock can move, but
            # keep the repeated timestamp out of the observable window.
            dt = 1/self.limits.control_hz
            self.samples.clear()
        else:
            dt = min(.25, dt)
        p, t = state.pose, self.target
        error = (t.x-p.x, t.y-p.y, t.z-p.z)
        # Velocity feedback damps drift instead of chasing a moving reference.
        desired = bounded(tuple(self.limits.position_gain*e-v for e,v in zip(error,state.velocity_world)), self.speed)
        velocity = self.translation.step(desired, dt)
        yaw_error = wrap_yaw_deg(t.yaw_deg-p.yaw_deg)
        yaw_rate = heading_rate_deg_s(
            yaw_error, state.yaw_rate_deg_s,
            gain=self.limits.heading.heading_hold_gain,
            maximum_rate_dps=self.limits.max_yaw_rate_deg_s,
            acceleration_dps2=self.limits.max_yaw_acceleration_deg_s2,
            jerk_dps3=self.tracking.max_yaw_jerk_deg_s3,
            damping=self.limits.heading.heading_rate_damping,
            deadband_deg=self.limits.heading.heading_deadband_deg,
            rate_deadband_dps=self.limits.heading.heading_rate_deadband_dps,
        )
        rate = self.rotation.step((yaw_rate,), dt)[0]
        self.vehicle.hold_velocity(velocity, rate, 1/self.limits.control_hz)
        if frozen:
            return
        self.samples.append((state.timestamp_ns, p))
        window_ns = self.limits.settle_time_s*1e9
        while len(self.samples) > 2 and self.samples[1][0] <= state.timestamp_ns-window_ns:
            self.samples.popleft()
        if now-self.last_diagnostic >= 1:
            logger.debug("Hold sample: position_error=%.3fm speed=%.3fm/s yaw_error=%.3fdeg "
                         "yaw_rate=%.3fdeg/s yaw_cmd=%.3fdeg/s observable=%s",
                        math.hypot(*error), math.hypot(*state.velocity_world), abs(yaw_error),
                        state.yaw_rate_deg_s if state.yaw_rate_deg_s is not None else float('nan'),
                        rate, self.observable)
            self.last_diagnostic = now

    @property
    def observable(self):
        if not self.active or len(self.samples) < 2:
            return False
        if self.samples[-1][0]-self.samples[0][0] < self.limits.settle_time_s*1e9:
            return False
        t = self.target
        # Test the whole window, not net displacement after a complete circle.
        positions = [(p.x,p.y,p.z) for _,p in self.samples]
        return (all(math.dist(v,(t.x,t.y,t.z)) <= self.limits.position_tolerance_m for v in positions)
                and all(math.dist(a,b) <= self.limits.position_tolerance_m for a in positions for b in positions)
                and all(abs(wrap_yaw_deg(p.yaw_deg-t.yaw_deg)) <= self.limits.yaw_tolerance_deg
                        for _,p in self.samples)
                and all(abs(wrap_yaw_deg(a.yaw_deg-b.yaw_deg)) <= self.limits.yaw_tolerance_deg
                        for _,a in self.samples for _,b in self.samples))

    def contains_pose(self, pose: WorldPose) -> bool:
        """Use the same reference bounds for the actual image-time pose."""
        t = self.target
        return bool(self.active and t is not None and
            math.dist((pose.x,pose.y,pose.z),(t.x,t.y,t.z)) <= self.limits.position_tolerance_m and
            abs(wrap_yaw_deg(pose.yaw_deg-t.yaw_deg)) <= self.limits.yaw_tolerance_deg)

    def wait(self, *, budget=None, checkpoint=None):
        self._owner()
        began = self.clock()
        while self.clock()-began < self.timeout_s:
            tick_started = self.clock()
            if checkpoint is not None:
                checkpoint()
            if budget is not None and budget.remaining(self.clock()) <= 0:
                raise BudgetExceeded("deadline during position hold")
            self.tick()
            if self.observable:
                logger.info("Hold confirmed: reference_z=%.3f actual_z=%.3f", self.target.z,self.state.pose.z)
                return
            self.sleep(max(.001,1/self.limits.control_hz-(self.clock()-tick_started)))
        raise ExecutionError("position hold did not stabilize before the motion timeout")

    def climb_by(self, metres: float) -> None:
        """Hold the current position shifted up by ``metres``.

        The native takeoff only frees the vehicle from the ground; the operating
        height comes from this climb, under the same hold the rest of the flight
        uses. NED z grows downward, so a higher hold is a smaller z.
        """
        extra = float(metres)
        if extra <= 0:
            return
        pose = self.vehicle.stream_state().pose
        self.start(WorldPose(pose.x, pose.y, pose.z-extra, pose.yaw_deg))
        self.wait()
        logger.info("Takeoff climb complete: extra=%.2fm z=%.2f", extra, self.state.pose.z)