"""流执行的记账：实测位姿差分、动作序列、剩余轨迹与时间线。

取帧、窗口轮询与发布都不在这里：这些由任务层在两次 ``run_stream`` 之间做。
"""

from __future__ import annotations

import math
import time
from collections import deque

from planner.domain import ExecutionRecord
from planner.domain import RelativeTrajectory
from planner.domain import resolve_yaw_delta_deg
from planner.errors import ExecutionError

from .tracking import measured_action
from ..stream_timeline import StreamTimeline


class StreamRecorder:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.timeline = StreamTimeline()
        self.queue = self.stage_token = None
        self.state = self.last_state = None
        self.yaw = None
        self.initial = None
        self.record = None
        self.actions = []
        self.path_length_m = 0.0

    def begin(self, queue, stage_token) -> None:
        self.queue, self.stage_token = queue, stage_token
        self.timeline.clear()
        self.state = self.last_state = None
        self.yaw = None
        self.initial = None
        self.record = None
        self.actions = []
        self.path_length_m = 0.0

    def reset(self, at) -> None:
        """从 ``at`` 重开执行历史；累计路程与时间线不清。

        碰撞退避结束后用：失败的段不能给重规划当锚点，而 ``path_length_m``
        还要给遥测与卡死看门狗继续用。
        """
        self.state = self.last_state = None
        self.yaw = None
        self.initial = at
        self.actions = []
        self.record = None

    def sample(self, state):
        """记录一次实测状态，返回连续偏航与位姿差分构成的执行记录。"""
        if self.state is None:
            self.yaw = state.pose.yaw_deg
            self.initial = state.pose
        else:
            if state.timestamp_ns < self.state.timestamp_ns:
                raise ExecutionError("simulator clock moved backwards during stream")
            delta = resolve_yaw_delta_deg(self.state.pose.yaw_deg, state.pose.yaw_deg)
            self.yaw += delta
            action = measured_action(self.state.pose, state.pose, delta)
            if math.hypot(action.dx, action.dy, action.dz) > 1e-6 or abs(delta) > 1e-6:
                self.actions.append(action)
                self.path_length_m += math.hypot(action.dx, action.dy, action.dz)
        self.last_state, self.state = self.state, state
        self.record = ExecutionRecord(
            self.initial, state.pose, RelativeTrajectory(tuple(self.actions[-200:]))
        )
        # 参考起点必须与保留的动作前缀一致：超出窗口时在位姿检查点重开，
        # 不截断增量还留着无关起点。
        if len(self.actions) > 200:
            self.actions = []
            self.initial = state.pose
            self.record = ExecutionRecord(state.pose, state.pose, RelativeTrajectory(()))
        self.timeline.record(
            state.timestamp_ns, self.stage_token, self.queue, self.record
        )
        return state
