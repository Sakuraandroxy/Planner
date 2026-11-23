"""连续路径的队列跟随器：消费航点、算期望命令、记这一拍的账。

``next`` 只做两件事：把已到达的航点从队列里消耗掉，给出这一拍的世界系期望命令。
它不取帧、不轮询规划窗、不发布遥测；那些由任务层在两次 ``run_stream`` 之间做。
队列空或暂时没有可消费航点时返回 ``None``，由执行层停止并交还任务层。

``tick`` 是同一条消费路径的自驱版本：采样、暂停判定、下发命令、记执行记录与遥测
都在这里，服务视觉目标那套自己排节的连续路径。
"""

from __future__ import annotations

import math
import time
from collections import deque

from planner.domain import (
    ContinuousTrackingSettings,
    ExecutionRecord,
    RelativeTrajectory,
    resolve_yaw_delta_deg,
)
from planner.errors import ExecutionError

from ..stream_timeline import StreamTimeline
from ..telemetry import ControlTelemetry
from .continuous import ContinuousFollower
from .continuous_reference import is_rotation
from .tracking import command_for, measured_action


class QueueFollower:
    def __init__(
        self,
        execution,
        streaming,
        limits,
        speed_mps,
        clock=time.monotonic,
        waypoint_timeout_s: float = 60.0,
        settings: ContinuousTrackingSettings = ContinuousTrackingSettings(),
    ):
        self.execution, self.streaming = execution, streaming
        self.limits, self.clock = limits, clock
        self.waypoint_timeout_s = float(waypoint_timeout_s)
        self.follower = ContinuousFollower(speed_mps, limits, settings)
        self.timeline = StreamTimeline()
        self.queue = self.stage_token = None
        self.state = self.last_state = None
        self.yaw = self.goal_yaw = self.head_id = None
        self.head_started = None
        self.last_ns = None
        self.consumed: tuple[int, ...] = ()
        self.initial = None
        self.record = None
        self.actions = []
        self.path_length_m = 0.0
        self.holding = True
        self.paused = None
        self.telemetry = deque(maxlen=12000)
        self.telemetry_dropped = 0
        self.last_telemetry_ns = 0

    @property
    def speed_mps(self) -> float:
        return self.follower.speed

    def begin(self, queue, stage_token=None) -> None:
        """绑定本阶段的队列视图，从当前队头重新起算。"""
        self.queue, self.stage_token = queue, stage_token
        self.timeline.clear()
        self.state = self.last_state = None
        self.yaw = None
        self.goal_yaw = self.head_id = None
        self.head_started = self.clock()
        self.last_ns = None
        self.consumed = ()
        self.initial = None
        self.record = None
        self.actions = []
        self.path_length_m = 0.0
        self.holding = True
        self.paused = None
        self.telemetry.clear()
        self.telemetry_dropped, self.last_telemetry_ns = 0, 0
        self.follower.reset()

    def reset(self) -> None:
        """丢掉当前航点的偏航目标；队列本身不动。"""
        self.goal_yaw = self.head_id = None
        self.follower.reset()

    def sample(self):
        """记一次实测状态，返回它。执行记录与时间线都从这一拍推进。"""
        self.execution.checkpoint()
        state = self.streaming.stream_state()
        if self.state is None:
            self.yaw = state.pose.yaw_deg
            self.initial = state.pose
        else:
            if state.timestamp_ns < self.state.timestamp_ns:
                raise ExecutionError(
                    "simulator clock moved backwards during stream"
                )
            delta = resolve_yaw_delta_deg(
                self.state.pose.yaw_deg, state.pose.yaw_deg
            )
            self.yaw += delta
            action = measured_action(self.state.pose, state.pose, delta)
            if (
                math.hypot(action.dx, action.dy, action.dz) > 1e-6
                or abs(delta) > 1e-6
            ):
                self.actions.append(action)
                self.path_length_m += math.hypot(
                    action.dx, action.dy, action.dz
                )
        self.last_state, self.state = self.state, state
        self.record = ExecutionRecord(
            self.initial, state.pose, RelativeTrajectory(tuple(self.actions[-200:]))
        )
        # 参考起点必须与保留的动作前缀一致：超出窗口时在位姿检查点重开，
        # 不截断增量还留着无关起点。
        if len(self.actions) > 200:
            self.actions = []
            self.initial = state.pose
            self.record = ExecutionRecord(
                state.pose, state.pose, RelativeTrajectory(())
            )
        self.timeline.record(
            state.timestamp_ns, self.stage_token, self.queue, self.record
        )
        return state

    def _head_goal(self, state, unwrapped_yaw: float) -> None:
        if not self.queue.items:
            return
        head = self.queue.items[0]
        if self.head_id == head.waypoint_id:
            return
        if self.goal_yaw is None:
            branch = unwrapped_yaw + resolve_yaw_delta_deg(
                state.pose.yaw_deg, head.start.yaw_deg
            )
            self.goal_yaw = branch + head.yaw_delta_deg
        else:
            self.goal_yaw += head.yaw_delta_deg
        self.head_id = head.waypoint_id
        self.head_started = self.clock()

    def _step(self, state, unwrapped_yaw_deg: float):
        """消费已到达的航点，算出这一拍的期望命令。

        返回 ``(command, sample_dt, velocity, rate)``；队列空或卡在停稳边界上时
        ``command`` 为 ``None``。
        """
        queue = self.queue
        self.consumed = ()
        if queue is None or not queue.items:
            return None, 0.0, None, None
        consumed = []
        while queue.items:
            self._head_goal(state, unwrapped_yaw_deg)
            items = queue.items
            # 队尾与原地转向之前先停稳，否则冲过端点再回头。
            stop_boundary = len(items) == 1 or is_rotation(items[1])
            if (
                stop_boundary
                and math.hypot(*state.velocity_world) > self.limits.stopped_speed_mps
            ):
                break
            if not queue.consume_stream(
                state.pose, unwrapped_yaw_deg, self.goal_yaw
            ):
                break
            consumed.append(self.head_id)
        self.consumed = tuple(consumed)
        if not queue.items:
            self.goal_yaw = self.head_id = None
            self.follower.reset()
            return None, 0.0, None, None
        # 采样驱动的 ``tick`` 以上一拍采样为基准；只调 ``next`` 的路径以这次调用为
        # 基准，视觉路径上两者同值。
        base_ns = (
            self.last_state.timestamp_ns
            if self.last_state is not None
            else self.last_ns
        )
        sample_dt = (
            (state.timestamp_ns - base_ns) / 1e9
            if base_ns is not None
            else 1 / self.limits.control_hz
        )
        if self.clock() - self.head_started >= self.waypoint_timeout_s:
            raise ExecutionError(
                "continuous waypoint timed out before position/yaw progress"
            )
        dt = max(0.001, min(0.25, sample_dt))
        velocity, rate = self.follower.sample(
            queue.items, state.pose, unwrapped_yaw_deg, self.goal_yaw, dt
        )
        self.last_ns = state.timestamp_ns
        return (
            command_for(
                self.streaming.motion_capability(),
                velocity,
                rate,
                state.pose.yaw_deg,
            ),
            sample_dt,
            velocity,
            rate,
        )

    def next(self, state, unwrapped_yaw_deg: float):
        """``StreamGoalSource``：只给命令，不下发、不记账。"""
        return self._step(state, unwrapped_yaw_deg)[0]

    def tick(self, pause_reason=None, *, sample=True):
        """自驱一拍：采样、暂停判定、下发命令、记账；返回这一拍消耗的航点。"""
        state = self.sample() if sample else self.state
        if self.paused is not None:
            return ()
        reason = pause_reason(state.pose) if pause_reason is not None else None
        if reason:
            self.pause(reason)
            return ()
        command, sample_dt, velocity, rate = self._step(state, self.yaw)
        self.timeline.record(
            state.timestamp_ns, self.stage_token, self.queue, self.record
        )
        if command is None:
            if not self.holding:
                self.execution.stop()
                self.holding = True
                self.goal_yaw = self.head_id = None
                self.follower.reset()
            return self.consumed
        self.streaming.stream_command(command, 1 / self.limits.control_hz)
        self._record_control(sample_dt, velocity, rate)
        self.holding = False
        return self.consumed

    def _record_control(self, dt, velocity, rate):
        state, follower = self.state, self.follower
        if state.timestamp_ns - self.last_telemetry_ns < 100_000_000:
            return
        if len(self.telemetry) == self.telemetry.maxlen:
            self.telemetry_dropped += 1
        self.telemetry.append(
            ControlTelemetry(
                state.timestamp_ns,
                state.observed_monotonic,
                dt,
                self.head_id,
                len(self.queue.items),
                velocity,
                state.velocity_world,
                follower.translation.acceleration,
                rate,
                follower.rotation.acceleration[0],
                self.goal_yaw - self.yaw,
                follower.reference.distance_m,
                follower.reference.aligning,
                state.pitch_deg,
                state.roll_deg,
            )
        )
        self.last_telemetry_ns = state.timestamp_ns

    def drain_telemetry(self):
        samples, dropped = tuple(self.telemetry), self.telemetry_dropped
        self.telemetry.clear()
        self.telemetry_dropped = 0
        return samples, dropped

    def pause(self, reason):
        self.execution.check_deadline()
        self._head_goal(self.state, self.yaw)
        stopped = self.execution.stop()
        if stopped.errors:
            raise ExecutionError("failed to stop continuous execution")
        self.sample()
        self.paused = self.queue.pause_stream(
            self.stage_token, self.state, self.yaw, self.goal_yaw, reason
        )
        self.timeline.record(
            self.state.timestamp_ns, self.stage_token, self.queue, self.record
        )
        self.holding = True
        self.follower.reset(self.state.velocity_world)
        return self.paused

    def resume(self) -> None:
        self.execution.check_deadline()
        if self.paused is None:
            return
        self.goal_yaw = self.queue.restore_stream(
            self.paused, self.stage_token
        )
        self.head_id = (
            self.queue.items[0].waypoint_id if self.queue.items else None
        )
        self.paused = None
        self.head_started = self.clock()
        self.follower.reset(self.state.velocity_world)
