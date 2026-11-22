"""连续路径的队列跟随器：消费航点、算期望命令，别的都不做。

``next`` 只做两件事：把已到达的航点从队列里消耗掉，给出这一拍的世界系期望命令。
它不取帧、不轮询规划窗、不发布遥测；那些由任务层在两次 ``run_stream`` 之间做。
队列空或暂时没有可消费航点时返回 ``None``，由执行层停止并交还任务层。
"""

from __future__ import annotations

import math
import time

from planner.domain import ContinuousTrackingSettings, resolve_yaw_delta_deg
from planner.errors import ExecutionError

from .continuous import ContinuousFollower
from .continuous_reference import is_rotation
from .tracking import command_for


class QueueFollower:
    def __init__(
        self,
        streaming,
        limits,
        speed_mps,
        clock=time.monotonic,
        waypoint_timeout_s: float = 60.0,
        settings: ContinuousTrackingSettings = ContinuousTrackingSettings(),
    ):
        self.streaming = streaming
        self.limits, self.clock = limits, clock
        self.waypoint_timeout_s = float(waypoint_timeout_s)
        self.follower = ContinuousFollower(speed_mps, limits, settings)
        self.queue = None
        self.goal_yaw = self.head_id = None
        self.head_started = None
        self.last_ns = None
        self.consumed: tuple[int, ...] = ()

    @property
    def speed_mps(self) -> float:
        return self.follower.speed

    def begin(self, queue) -> None:
        """绑定本阶段的队列视图，从当前队头重新起算。"""
        self.queue = queue
        self.goal_yaw = self.head_id = None
        self.head_started = self.clock()
        self.last_ns = None
        self.consumed = ()
        self.follower.reset()

    def reset(self) -> None:
        """丢掉当前航点的偏航目标；队列本身不动。"""
        self.goal_yaw = self.head_id = None
        self.follower.reset()

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

    def next(self, state, unwrapped_yaw_deg: float):
        queue = self.queue
        self.consumed = ()
        if queue is None or not queue.items:
            return None
        consumed = []
        while queue.items:
            self._head_goal(state, unwrapped_yaw_deg)
            items = queue.items
            # 队尾与原地转向之前先停稳，否则冲过端点再回头。
            stop_boundary = len(items) == 1 or is_rotation(items[1])
            if stop_boundary and math.hypot(*state.velocity_world) > self.limits.stopped_speed_mps:
                break
            if not queue.consume_stream(state.pose, unwrapped_yaw_deg, self.goal_yaw):
                break
            consumed.append(self.head_id)
        self.consumed = tuple(consumed)
        if not queue.items:
            self.goal_yaw = self.head_id = None
            self.follower.reset()
            return None
        sample_dt = (
            (state.timestamp_ns - self.last_ns) / 1e9
            if self.last_ns is not None
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
        return command_for(
            self.streaming.motion_capability(),
            velocity,
            rate,
            state.pose.yaw_deg,
        )
