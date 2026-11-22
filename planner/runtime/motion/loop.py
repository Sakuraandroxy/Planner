"""把目标变成合法命令的两条控制循环。

``run_segment`` 按段走一条 ``MotionTrajectory``：以实测位姿重锚、预算预检、
逐段执行、停稳后填回执，任何一步失败都带着已完成段数与剩余轨迹上抛。
``run_stream`` 是连续路径的一拍：读一次状态、向目标源要期望命令、限幅后下发。
它可以重复调用，返回即交给任务层；取帧、窗口轮询与暂停判定都在两次调用之间，
调用方不调用它就是暂停。两个周期只住在这里：段拍的 ``SEGMENT_PERIOD_S`` 与
流拍的 ``1 / MotionLimits.control_hz``。
"""

from __future__ import annotations

import math
import time
from dataclasses import replace

from planner.domain import (
    ExecutionBudget,
    ExecutionReceipt,
    ExecutionRecord,
    MotionLimits,
    MotionTrajectory,
    RelativePoseDelta,
    RelativeTrajectory,
    resolve_yaw_delta_deg,
    wrap_yaw_deg,
)
from planner.errors import BudgetExceeded, ExecutionError, MotionPaused

#: 段执行一拍。固定值，不进配置（裁决 R9）。
SEGMENT_PERIOD_S = 0.030


def measured_action(start, end, yaw):
    """实测位移换算到起点航向系，附上带符号的偏航增量。"""
    angle = math.radians(start.yaw_deg)
    x, y = end.x - start.x, end.y - start.y
    return RelativePoseDelta(
        math.cos(angle) * x + math.sin(angle) * y,
        -math.sin(angle) * x + math.cos(angle) * y,
        end.z - start.z,
        yaw,
    )


def _remaining(current, actual_yaw, segment, tail):
    """保留原始世界端点与带符号偏航目标，从实测位姿重锚。"""
    original_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
        segment.start.yaw_deg, segment.end.yaw_deg, segment.yaw_delta_deg
    )
    remaining = replace(segment, start=current, yaw_delta_deg=original_yaw - actual_yaw)
    return MotionTrajectory((remaining, *tail))


class ControlLoop:
    def __init__(
        self,
        execution,
        streaming,
        *,
        synthesizer=None,
        limiter=None,
        recovery=None,
        window=None,
        limits=None,
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        self.execution = execution
        self.streaming = streaming
        self.synthesizer = synthesizer
        self.limiter = limiter
        self.recovery = recovery
        self.window = window
        self.limits = limits if limits is not None else getattr(
            streaming, "tracking_limits", None
        )
        if self.limits is None:
            self.limits = MotionLimits()
        self.clock, self.sleep = clock, sleep
        self.state = None
        self.holding = True
        self._running = False
        self._last_ns = None
        self._yaw = None
        self._pose_yaw = None

    @property
    def period(self) -> float:
        """流拍长：由运动包络的 ``control_hz`` 给出。"""
        return 1.0 / self.limits.control_hz

    def segment_sink(self):
        return self.streaming.stream_command

    def stream_sink(self):
        return self.streaming.stream_command

    def _guard(self, budget: ExecutionBudget | None) -> None:
        self.execution.checkpoint()
        if budget is not None and budget.remaining(self.clock()) <= 0:
            raise BudgetExceeded("stage time budget exhausted")

    def execute(self, trajectory: MotionTrajectory) -> ExecutionRecord:
        self.execution.checkpoint()
        start = self.streaming.current_pose()
        receipt = self.run_segment(trajectory, self.segment_sink())
        return ExecutionRecord(
            start,
            receipt.final_pose,
            RelativeTrajectory(receipt.measured_actions),
            receipt.paused,
        )

    def run_segment(
        self,
        trajectory: MotionTrajectory,
        sink,
        *,
        budget: ExecutionBudget | None = None,
        pause_when=None,
    ) -> ExecutionReceipt:
        self.execution.assert_owner()
        if self._running:
            raise ExecutionError("nested motion execution is not allowed")
        if not trajectory.segments:
            raise ExecutionError("cannot execute an empty motion trajectory")
        self._running = True
        began = self.clock()
        completed = 0
        measured_chords = 0.0
        actions = []
        active = None
        previous = None
        observed_yaw = None
        measurement_start_yaw = None
        try:
            self.execution.checkpoint()
            previous = self.streaming.current_pose()
            for index, segment in enumerate(trajectory.segments):
                self.execution.checkpoint()
                if budget is not None and budget.remaining(self.clock()) <= 0:
                    raise BudgetExceeded("deadline before segment")
                # 跟踪器沿用原始段的角分支，与实测位姿可能差 360 度。
                measurement_start_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
                    segment.start.yaw_deg, previous.yaw_deg
                )
                observed_yaw = measurement_start_yaw
                active = segment

                def checkpoint(pose, unwrapped_yaw):
                    nonlocal observed_yaw
                    observed_yaw = unwrapped_yaw
                    self.execution.check_safety()
                    return pause_when(pose) if pause_when is not None else False

                try:
                    self.streaming.execute_segment(
                        segment, budget=budget, pose_checkpoint=checkpoint
                    )
                except MotionPaused as paused:
                    stop = self.execution.stop()
                    if stop.errors:
                        raise ExecutionError(
                            "failed to cancel/hover after execution pause"
                        )
                    current = self.streaming.current_pose()
                    if self.execution.collision_state():
                        raise ExecutionError("collision while pausing execution")
                    actual_yaw = paused.unwrapped_yaw_deg + wrap_yaw_deg(
                        current.yaw_deg - paused.pose.yaw_deg
                    )
                    actions.append(
                        measured_action(previous, current, actual_yaw - measurement_start_yaw)
                    )
                    measured_chords += math.dist(
                        (previous.x, previous.y, previous.z),
                        (current.x, current.y, current.z),
                    )
                    remaining = _remaining(
                        current, actual_yaw, segment, trajectory.segments[index + 1 :]
                    )
                    return ExecutionReceipt(
                        completed,
                        current,
                        self.clock() - began,
                        measured_chords,
                        True,
                        remaining,
                        tuple(actions),
                    )
                completed += 1
                current = self.streaming.current_pose()
                yaw = observed_yaw + wrap_yaw_deg(current.yaw_deg - observed_yaw) - measurement_start_yaw
                actions.append(measured_action(previous, current, yaw))
                measured_chords += math.dist(
                    (previous.x, previous.y, previous.z),
                    (current.x, current.y, current.z),
                )
                previous = current
                active = None
                self.execution.checkpoint()
                if self.execution.collision_state():
                    raise ExecutionError("AirSim reported a collision")
                if budget is not None and budget.remaining(self.clock()) <= 0:
                    raise BudgetExceeded("deadline after segment")
            return ExecutionReceipt(
                completed,
                self.streaming.current_pose(),
                self.clock() - began,
                measured_chords,
                measured_actions=tuple(actions),
            )
        except (Exception, KeyboardInterrupt) as exc:
            stop = self.execution.stop()
            try:
                current = self.streaming.current_pose()
                if active is not None and previous is not None:
                    actual_yaw = observed_yaw + wrap_yaw_deg(current.yaw_deg - observed_yaw)
                    actions.append(
                        measured_action(previous, current, actual_yaw - measurement_start_yaw)
                    )
                    measured_chords += math.dist(
                        (previous.x, previous.y, previous.z),
                        (current.x, current.y, current.z),
                    )
                    remaining = _remaining(
                        current, actual_yaw, active, trajectory.segments[completed + 1 :]
                    )
                else:
                    remaining = MotionTrajectory(trajectory.segments[completed:])
                receipt = ExecutionReceipt(
                    completed,
                    current,
                    self.clock() - began,
                    measured_chords,
                    remaining=remaining,
                    measured_actions=tuple(actions),
                )
            except Exception:
                receipt = None
            if isinstance(exc, (ExecutionError, KeyboardInterrupt)):
                exc.receipt, exc.stop_result = receipt, stop
                raise
            wrapped = ExecutionError(str(exc))
            wrapped.receipt, wrapped.stop_result = receipt, stop
            raise wrapped from exc
        finally:
            self._running = False

    def run_stream(self, goals, sink, *, deadline_monotonic=None) -> None:
        """连续路径的一拍。

        目标源返回 ``None`` 或队列为空时正常返回，并在返回前调 ``execution.stop()``；
        ``deadline_monotonic`` 到期抛 ``BudgetExceeded``，检查点抛 ``ExecutionError``，
        两者都由任务层决定怎么收尾。
        """
        self.execution.assert_owner()
        budget = (
            None if deadline_monotonic is None else ExecutionBudget(deadline_monotonic)
        )
        self._guard(budget)
        state = self.streaming.stream_state()
        if self._last_ns is not None and state.timestamp_ns < self._last_ns:
            raise ExecutionError("simulator clock moved backwards during stream")
        self._last_ns = state.timestamp_ns
        if self._yaw is None:
            self._yaw = state.pose.yaw_deg
        else:
            self._yaw += resolve_yaw_delta_deg(self._pose_yaw, state.pose.yaw_deg)
        self._pose_yaw = state.pose.yaw_deg
        self.state = state
        command = goals.next(state, self._yaw)
        if command is None:
            if not self.holding:
                self.execution.stop()
                self.holding = True
            return
        if self.limiter is not None:
            command = self.limiter.limit(command, state, self.period)
        sink(command, self.period)
        self.holding = False
