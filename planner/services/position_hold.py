"""Owner-thread position holding shared by observation, task end and idle waits."""
from collections import deque
import logging
import math
import threading
import time

from planner.domain.pose import wrap_yaw_deg
from planner.domain.motion import MotionLimits, ContinuousTrackingSettings
from planner.domain.pose import WorldPose
from planner.domain.streaming import TimedVehicleState
from planner.ports.position_hold_vehicle import PositionHoldVehicle
from planner.errors import BudgetExceeded, ExecutionError
from planner.services.motion.command_smoothing import CommandSmoother, bounded
from planner.services.motion.hold_handoff import translation_handoff

logger = logging.getLogger(__name__)


class PositionHold:
    def __init__(self, vehicle: PositionHoldVehicle, limits: MotionLimits,
                 tracking: ContinuousTrackingSettings, speed_mps: float, timeout_s: float,
                 *, clock=time.monotonic, sleep=time.sleep):
        self.vehicle, self.limits = vehicle, limits
        self.speed, self.timeout_s, self.clock, self.sleep = speed_mps, timeout_s, clock, sleep
        self.translation = CommandSmoother(limits.max_acceleration_mps2, tracking.max_jerk_mps3)
        self.rotation = CommandSmoother(limits.max_yaw_acceleration_deg_s2, tracking.max_yaw_jerk_deg_s3)
        self.owner = threading.get_ident()
        self.target: WorldPose | None = None
        self.state: TimedVehicleState | None = None
        self.active = False
        self.samples = deque()
        self.observation_samples = deque()
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
        self.observation_samples.clear()
        self.state = state
        self.active = True
        self.last_tick = self.last_diagnostic = -math.inf
        logger.info("Hold reference NED: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                    self.target.x, self.target.y, self.target.z, self.target.yaw_deg)

    def release(self):
        self._owner()
        self.active = False
        self.samples.clear()
        self.observation_samples.clear()

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
        if dt <= 0:
            self.samples.clear()
            self.observation_samples.clear()
            return  # A frozen simulator cannot provide an observable hold window.
        measured_yaw_rate = abs(wrap_yaw_deg(state.pose.yaw_deg-previous.pose.yaw_deg))/dt
        dt = min(.25, dt)
        p, t = state.pose, self.target
        error = (t.x-p.x, t.y-p.y, t.z-p.z)
        # Velocity feedback damps drift instead of chasing a moving reference.
        desired = bounded(tuple(self.limits.position_gain*e-v for e,v in zip(error,state.velocity_world)), self.speed)
        velocity = self.translation.step(desired, dt)
        yaw_error = wrap_yaw_deg(t.yaw_deg-p.yaw_deg)
        yaw_rate = max(-self.limits.max_yaw_rate_deg_s,
                       min(self.limits.max_yaw_rate_deg_s, self.limits.yaw_gain*yaw_error))
        rate = self.rotation.step((yaw_rate,), dt)[0]
        self.vehicle.hold_velocity(velocity, rate, 1/self.limits.control_hz)
        self.samples.append((state.timestamp_ns, p))
        self.observation_samples.append((state.timestamp_ns, math.hypot(*state.velocity_world), measured_yaw_rate))
        window_ns = self.limits.settle_time_s*1e9
        while len(self.samples) > 2 and self.samples[1][0] <= state.timestamp_ns-window_ns:
            self.samples.popleft()
        while len(self.observation_samples) > 2 and self.observation_samples[1][0] <= state.timestamp_ns-window_ns:
            self.observation_samples.popleft()
        if now-self.last_diagnostic >= 1:
            logger.debug("Hold sample: position_error=%.3fm speed=%.3fm/s yaw_error=%.3fdeg observable=%s",
                        math.hypot(*error), math.hypot(*state.velocity_world), abs(yaw_error), self.observable)
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

    @property
    def observation_ready(self) -> bool:
        """Low motion is required for formal images, not for every hold user."""
        samples = self.observation_samples
        return bool(self.observable and len(samples) >= 2 and
            samples[-1][0]-samples[0][0] >= self.limits.settle_time_s*1e9 and
            all(speed <= self.limits.stopped_speed_mps and rate <= self.limits.stopped_yaw_rate_deg_s
                for _,speed,rate in samples))

    def wait(self, *, budget=None, checkpoint=None):
        self._wait(lambda: self.observable, "Hold", budget=budget, checkpoint=checkpoint)

    def wait_for_observation(self, *, budget=None, checkpoint=None):
        self._wait(lambda: self.observation_ready, "Observation hold", budget=budget, checkpoint=checkpoint)

    def _wait(self, ready, description, *, budget=None, checkpoint=None):
        self._owner()
        began = self.clock()
        while self.clock()-began < self.timeout_s:
            tick_started = self.clock()
            if checkpoint is not None:
                checkpoint()
            if budget is not None and budget.remaining(self.clock()) <= 0:
                raise BudgetExceeded("deadline during position hold")
            self.tick()
            if ready():
                logger.info("%s confirmed: reference_z=%.3f actual_z=%.3f", description,self.target.z,self.state.pose.z)
                return
            self.sleep(max(.001,1/self.limits.control_hz-(self.clock()-tick_started)))
        raise ExecutionError("position hold did not stabilize before the motion timeout")
