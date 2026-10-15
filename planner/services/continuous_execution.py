"""Single-owner queue follower. Model/capture workers never touch this service."""
import math
import time

from planner.domain.execution import ExecutionRecord
from planner.domain.pose import resolve_yaw_delta_deg
from planner.domain.trajectory import RelativeTrajectory
from planner.errors import ExecutionError
from planner.services.execution import ExecutionService
from planner.services.motion.continuous import ContinuousFollower
from planner.services.stream_timeline import StreamTimeline
from planner.ports.streaming_vehicle import StreamingVehicle


class ContinuousExecution:
    def __init__(self, execution: ExecutionService, vehicle: StreamingVehicle, limits, speed_mps,
                 clock=time.monotonic, waypoint_timeout_s=60.):
        self.execution, self.vehicle = execution, vehicle
        self.limits, self.clock = limits, clock
        self.follower = ContinuousFollower(speed_mps, limits)
        self.timeline = StreamTimeline()
        self.state = self.last_state = None
        self.yaw = self.goal_yaw = None
        self.head_id = None
        self.paused = None
        self.record = None
        self.actions = []
        self.path_length_m = 0.0
        self.holding = True
        self.waypoint_timeout_s = waypoint_timeout_s
        self.head_started = clock()

    def begin(self, queue, stage_token):
        self.execution.check_deadline()
        self.queue, self.stage_token = queue, stage_token
        self.timeline.clear()
        self.state = self.last_state = None
        self.yaw = self.goal_yaw = None
        self.head_id, self.paused, self.record = None, None, None
        self.actions, self.path_length_m = [], 0.0
        self.holding = True
        self.sample()
        self.follower.reset()

    def sample(self):
        self.execution.checkpoint()
        state = self.vehicle.stream_state()
        if self.state is None:
            self.yaw = state.pose.yaw_deg
            self.initial = state.pose
        else:
            if state.timestamp_ns < self.state.timestamp_ns:
                raise ExecutionError("simulator clock moved backwards during stream")
            delta = resolve_yaw_delta_deg(self.state.pose.yaw_deg, state.pose.yaw_deg)
            self.yaw += delta
            action = ExecutionService._measured_action(self.state.pose, state.pose, delta)
            if math.hypot(action.dx, action.dy, action.dz) > 1e-6 or abs(delta) > 1e-6:
                self.actions.append(action)
                self.path_length_m += math.hypot(action.dx, action.dy, action.dz)
        self.last_state, self.state = self.state, state
        self.record = ExecutionRecord(self.initial, state.pose, RelativeTrajectory(tuple(self.actions[-200:])))
        # The reference start must match the retained action prefix. Keep a
        # bounded record by resetting at a measured checkpoint, never truncating
        # deltas while retaining an unrelated start pose.
        if len(self.actions) > 200:
            self.actions = []
            self.initial = state.pose
            self.record = ExecutionRecord(state.pose, state.pose, RelativeTrajectory(()))
        self.timeline.record(state.timestamp_ns, self.stage_token, self.queue, self.record)
        return state

    def _head_goal(self):
        if not self.queue.items:
            return
        head = self.queue.items[0]
        if self.head_id != head.waypoint_id:
            if self.goal_yaw is None:
                branch = self.yaw + resolve_yaw_delta_deg(self.state.pose.yaw_deg, head.start.yaw_deg)
                self.goal_yaw = branch+head.yaw_delta_deg
            else:
                self.goal_yaw += head.yaw_delta_deg
            self.head_id = head.waypoint_id
            self.head_started = self.clock()

    def tick(self, pause_reason=None, *, sample=True):
        state = self.sample() if sample else self.state
        if self.paused is not None:
            return ()
        reason = pause_reason(state.pose) if pause_reason is not None else None
        if reason:
            self.pause(reason)
            return ()
        consumed = []
        while self.queue.items:
            self._head_goal()
            if not self.queue.consume_stream(state.pose, self.yaw, self.goal_yaw):
                break
            consumed.append(self.head_id)
        self.timeline.record(state.timestamp_ns, self.stage_token, self.queue, self.record)
        if not self.queue.items:
            if not self.holding:
                self.execution.stop()
                self.holding = True
                self.goal_yaw = self.head_id = None
                self.follower.reset()
            return tuple(consumed)
        dt = (state.timestamp_ns-self.last_state.timestamp_ns)/1e9 if self.last_state else 1/self.limits.control_hz
        if self.clock()-self.head_started >= self.waypoint_timeout_s:
            raise ExecutionError("continuous waypoint timed out before position/yaw progress")
        dt = max(.001, min(.25, dt))
        velocity, rate = self.follower.sample(self.queue.items, state.pose, self.yaw, self.goal_yaw, dt)
        self.vehicle.stream_velocity(velocity, rate, 1/self.limits.control_hz)
        self.holding = False
        return tuple(consumed)

    def pause(self, reason):
        self.execution.check_deadline()
        self._head_goal()
        stopped = self.execution.stop()
        if stopped.errors:
            raise ExecutionError("failed to stop continuous execution")
        self.sample()
        self.paused = self.queue.pause_stream(self.stage_token, self.state, self.yaw, self.goal_yaw, reason)
        self.timeline.record(self.state.timestamp_ns, self.stage_token, self.queue, self.record)
        self.holding = True
        self.follower.reset()
        return self.paused

    def resume(self):
        self.execution.check_deadline()
        if self.paused is None:
            return
        self.goal_yaw = self.queue.restore_stream(self.paused, self.stage_token)
        self.head_id = self.queue.items[0].waypoint_id if self.queue.items else None
        self.paused = None
        self.head_started = self.clock()
        self.follower.reset()

