"""把目标变成合法命令的两条控制循环。

``run_segment`` 按段走一条 ``MotionTrajectory``：以实测位姿重锚、预算预检、逐拍
采样五次曲线、限幅后下发、停稳验证，任何一步失败都带着已完成段数与剩余轨迹上抛。
``run_stream`` 是连续路径的一拍：读一次状态、向目标源要期望命令、限幅后下发。
它可以重复调用，返回即交给任务层；取帧、窗口轮询与暂停判定都在两次调用之间，
调用方不调用它就是暂停。两个周期只住在这里：段拍的 ``SEGMENT_PERIOD_S`` 与
流拍的 ``1 / MotionLimits.control_hz``。
"""

from __future__ import annotations

import math
import time
from contextlib import nullcontext
from dataclasses import replace

from planner.domain import (
    ExecutionBudget,
    ExecutionReceipt,
    ExecutionRecord,
    MotionLimits,
    MotionTrajectory,
    RelativeTrajectory,
    resolve_yaw_delta_deg,
    wrap_yaw_deg,
)
from planner.errors import (
    BudgetExceeded,
    CollisionInterrupt,
    ExecutionError,
    MotionPaused,
)
from planner.ports import StreamGoalSource

from ..collision_recovery import CollisionRecovery
from .synchronized import duration_for
from .record import StreamRecorder
from .tracking import SegmentSynthesizer, command_for, measured_action

#: 段执行一拍。固定值，不进配置（裁决 R9）。
SEGMENT_PERIOD_S = 0.030

#: 段执行在停稳前允许花掉的最长时间，装配层按载具配置覆盖。
DEFAULT_MOTION_TIMEOUT_S = 60.0


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
        collision=None,
        window=None,
        limits=None,
        tracking=None,
        timeout_s=None,
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        self.execution = execution
        self.streaming = streaming
        self.limiter = limiter
        #: 碰撞退避；装配层在 ``motion.collision.enabled`` 关时传 ``None``。
        self.recovery = (
            CollisionRecovery(collision) if collision is not None else None
        )
        self.window = window
        self.limits = limits if limits is not None else getattr(
            streaming, "tracking_limits", None
        )
        if self.limits is None:
            self.limits = MotionLimits()
        self.synthesizer = synthesizer or SegmentSynthesizer(self.limits, tracking)
        self.timeout_s = (
            DEFAULT_MOTION_TIMEOUT_S if timeout_s is None else float(timeout_s)
        )
        self.clock, self.sleep = clock, sleep
        #: UE4 的取帧门：段执行每拍服务一次在途取帧请求，恢复后从实测位姿重锚。
        self.observation_gate = None
        self.recorder = StreamRecorder(clock)
        self.state = None
        self.holding = True
        self._running = False

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
        self._poll_contact()
        if budget is not None and budget.remaining(self.clock()) <= 0:
            raise BudgetExceeded("stage time budget exhausted")

    def _poll_contact(self) -> None:
        """读一次接触状态；有未处理的接触就抛 ``CollisionInterrupt``。"""
        if self.recovery is None:
            return
        self.recovery.note_progress(self.recorder.path_length_m)
        contact = self.recovery.poll(self.streaming.collision_state())
        if contact is not None:
            raise CollisionInterrupt(contact)

    def _recover(self, contact, *, clear_queue: bool) -> None:
        """停稳、按运动律后退 ``backoff_m * attempts``、作废推测输出。

        ``clear_queue`` 在连续路径上为真：授权队列、在途规划与执行历史都从恢复
        位姿重开，下一拍由任务层重新取帧补点。段执行由调用方持有的轨迹决定，
        队列不动，后退后把剩余轨迹交给 receipt。
        """
        self.execution.stop()
        plan = self.recovery.begin(contact, self.streaming.current_pose())
        self._back_off(plan)
        self.recovery.finish()
        self.holding = True
        self.synthesizer.reset()
        reset = getattr(self.limiter, "reset", None)
        if reset is not None:
            reset()
        if clear_queue and self.window is not None:
            self.window.interrupt()
        self.recorder.reset(self.streaming.current_pose())

    def _back_off(self, plan) -> None:
        """沿当前航向倒车 ``plan.distance_m``，按 ``SEGMENT_PERIOD_S`` 每拍下发。"""
        limits = self.limits
        period = SEGMENT_PERIOD_S
        if limits.max_speed_mps <= 0:
            raise ExecutionError("collision backoff requires a positive speed limit")
        expected = (
            plan.distance_m / limits.max_speed_mps
            + limits.max_speed_mps / limits.max_acceleration_mps2
        )
        maximum_steps = math.ceil(expected / period) * 3 + 10
        capability = self.streaming.motion_capability()
        sink = self.stream_sink()
        previous = self.streaming.current_pose()
        angle = math.radians(previous.yaw_deg)
        speed = travelled = 0.0
        for _ in range(maximum_steps):
            if travelled >= plan.distance_m:
                sink(
                    command_for(capability, (0.0, 0.0, 0.0), 0.0, previous.yaw_deg),
                    period,
                )
                return
            speed = min(
                limits.max_speed_mps, speed + limits.max_acceleration_mps2 * period
            )
            sink(
                command_for(
                    capability,
                    (-speed * math.cos(angle), -speed * math.sin(angle), 0.0),
                    0.0,
                    previous.yaw_deg,
                ),
                period,
            )
            current = self.streaming.current_pose()
            travelled += math.dist(
                (previous.x, previous.y, previous.z),
                (current.x, current.y, current.z),
            )
            previous = current
        raise ExecutionError("collision backoff made no measurable progress")

    def _check_budget(self, budget: ExecutionBudget | None, required_s: float = 0.0):
        if budget is not None and budget.remaining(self.clock()) <= required_s:
            raise BudgetExceeded("motion/task deadline exhausted")

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
        pose_checkpoint=None,
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

                if pose_checkpoint is not None:
                    checkpoint = pose_checkpoint

                    def checkpoint(pose, unwrapped_yaw):
                        nonlocal observed_yaw
                        observed_yaw = unwrapped_yaw
                        return pose_checkpoint(pose, unwrapped_yaw)
                else:

                    def checkpoint(pose, unwrapped_yaw):
                        nonlocal observed_yaw
                        observed_yaw = unwrapped_yaw
                        self.execution.check_deadline()
                        return pause_when(pose) if pause_when is not None else False

                try:
                    self._drive_segment(segment, sink, budget, checkpoint)
                except MotionPaused as paused:
                    stop = self.execution.stop()
                    if stop.errors:
                        raise ExecutionError(
                            "failed to cancel/hover after execution pause"
                        )
                    current = self.streaming.current_pose()
                    actual_yaw = paused.unwrapped_yaw_deg + wrap_yaw_deg(
                        current.yaw_deg - paused.pose.yaw_deg
                    )
                    actions.append(
                        measured_action(
                            previous, current, actual_yaw - measurement_start_yaw
                        )
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
                except CollisionInterrupt as interrupt:
                    contact_pose = self.streaming.current_pose()
                    self._recover(interrupt.contact, clear_queue=False)
                    current = self.streaming.current_pose()
                    actual_yaw = measurement_start_yaw + wrap_yaw_deg(
                        current.yaw_deg - contact_pose.yaw_deg
                    )
                    actions.append(
                        measured_action(
                            previous, current, actual_yaw - measurement_start_yaw
                        )
                    )
                    measured_chords += math.dist(
                        (previous.x, previous.y, previous.z),
                        (current.x, current.y, current.z),
                    )
                    remaining = _remaining(
                        current,
                        actual_yaw,
                        segment,
                        trajectory.segments[index + 1 :],
                    )
                    return ExecutionReceipt(
                        completed,
                        current,
                        self.clock() - began,
                        measured_chords,
                        remaining=remaining,
                        measured_actions=tuple(actions),
                    )
                completed += 1
                current = self.streaming.current_pose()
                yaw = (
                    observed_yaw
                    + wrap_yaw_deg(current.yaw_deg - observed_yaw)
                    - measurement_start_yaw
                )
                actions.append(measured_action(previous, current, yaw))
                measured_chords += math.dist(
                    (previous.x, previous.y, previous.z),
                    (current.x, current.y, current.z),
                )
                previous = current
                active = None
                self.execution.checkpoint()
                self._poll_contact()
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
                    actual_yaw = observed_yaw + wrap_yaw_deg(
                        current.yaw_deg - observed_yaw
                    )
                    actions.append(
                        measured_action(
                            previous, current, actual_yaw - measurement_start_yaw
                        )
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

    def _drive_segment(self, segment, sink, budget, pose_checkpoint) -> None:
        """按 ``SEGMENT_PERIOD_S`` 一拍走完一段，并在停稳窗口内验证到位。"""
        limits = self.limits
        speed = segment.target_speed_mps
        if speed is None:
            raise ExecutionError("smooth motion requires a speed limit")
        period = SEGMENT_PERIOD_S
        goal_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
            segment.start.yaw_deg, segment.end.yaw_deg, segment.yaw_delta_deg
        )
        start = self.streaming.current_pose()
        measured_yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
            segment.start.yaw_deg, start.yaw_deg
        )
        remaining_yaw = goal_yaw - measured_yaw
        duration = max(
            segment.duration_s or 0,
            duration_for(start, segment.end, speed, limits, yaw_delta_deg=remaining_yaw),
        )
        self._check_budget(budget, duration + limits.settle_time_s)
        if duration + limits.settle_time_s >= self.timeout_s:
            raise ExecutionError(
                f"smooth duration {duration:.2f}s exceeds available execution timeout"
            )
        # 把插值起点重锚到实测位姿，但保留原始世界终点与带符号偏航目标。
        active = replace(
            segment, start=start, duration_s=duration, yaw_delta_deg=remaining_yaw
        )
        self.synthesizer.reset()
        capability = self.streaming.motion_capability()
        gate = self.observation_gate
        generation = gate.generation if gate is not None else 0
        steps = 0
        previous_pose = start
        settled_since = None
        while True:
            guard = gate.step() if gate is not None else nullcontext(generation)
            with guard as current_generation:
                if current_generation != generation:
                    # 段内做过一次取帧：从实测位姿恢复，参考曲线与时长重算。
                    generation = current_generation
                    start = self.streaming.current_pose()
                    measured_yaw += resolve_yaw_delta_deg(
                        previous_pose.yaw_deg, start.yaw_deg
                    )
                    remaining_yaw = goal_yaw - measured_yaw
                    duration = duration_for(
                        start,
                        active.end,
                        speed,
                        limits,
                        yaw_delta_deg=remaining_yaw,
                    )
                    self._check_budget(budget, duration + limits.settle_time_s)
                    if duration + limits.settle_time_s >= self.timeout_s:
                        raise ExecutionError("resumed motion exceeds execution timeout")
                    active = replace(
                        active,
                        start=start,
                        duration_s=duration,
                        yaw_delta_deg=remaining_yaw,
                    )
                    self.synthesizer.reset()
                    steps = 0
                    previous_pose = start
                    settled_since = None
                self._check_budget(budget)
                self._poll_contact()
                elapsed = steps * period
                if elapsed >= self.timeout_s:
                    raise ExecutionError(
                        "synchronized motion timed out before position/yaw settled"
                    )
                actual = self.streaming.current_pose()
                if steps:
                    measured_yaw += resolve_yaw_delta_deg(
                        previous_pose.yaw_deg, actual.yaw_deg
                    )
                moved = math.dist(
                    (actual.x, actual.y, actual.z),
                    (previous_pose.x, previous_pose.y, previous_pose.z),
                )
                measured_speed = moved / period
                measured_yaw_rate = (
                    abs(wrap_yaw_deg(actual.yaw_deg - previous_pose.yaw_deg)) / period
                )
                goal = active.end
                distance = math.dist(
                    (actual.x, actual.y, actual.z), (goal.x, goal.y, goal.z)
                )
                yaw_error = abs(goal_yaw - measured_yaw)
                settled = (
                    elapsed >= duration
                    and distance <= limits.position_tolerance_m
                    and yaw_error <= limits.yaw_tolerance_deg
                    and measured_speed <= limits.stopped_speed_mps
                    and measured_yaw_rate <= limits.stopped_yaw_rate_deg_s
                )
                settled_since = (
                    (elapsed if settled_since is None else settled_since)
                    if settled
                    else None
                )
                if (
                    settled_since is not None
                    and elapsed - settled_since >= limits.settle_time_s
                ):
                    final = self.streaming.current_pose()
                    final_distance = math.dist(
                        (final.x, final.y, final.z), (goal.x, goal.y, goal.z)
                    )
                    final_yaw = abs(
                        goal_yaw
                        - (measured_yaw + wrap_yaw_deg(final.yaw_deg - actual.yaw_deg))
                    )
                    if (
                        final_distance > limits.position_tolerance_m
                        or final_yaw > limits.yaw_tolerance_deg
                    ):
                        raise ExecutionError(
                            "position/yaw drifted outside tolerance while hovering"
                        )
                    return
                if pose_checkpoint is not None and pose_checkpoint(
                    actual, measured_yaw
                ):
                    raise MotionPaused(actual, measured_yaw)
                velocity, yaw_rate = self.synthesizer.command(
                    active, elapsed, actual, period
                )
                if not all(math.isfinite(v) for v in (*velocity, yaw_rate, period)):
                    raise ExecutionError("non-finite control velocity or yaw rate")
                sink(
                    command_for(capability, velocity, yaw_rate, actual.yaw_deg),
                    period,
                )
                previous_pose = actual
                steps += 1

    def sample(self):
        """读一次实测状态并记进时间线；任务层的取帧与窗口都读这一次。

        前置条件：本阶段必须先调 ``self.recorder.begin(queue, stage_token)``。
        时间线要按模拟器时间记下当时的队列修订，没绑定队列就没有可记的修订。
        未调 ``begin`` 就调本方法抛 ``RuntimeError``。
        """
        if self.recorder.queue is None:
            raise RuntimeError(
                "control loop sample needs StreamRecorder.begin(queue, stage_token) "
                "called once at stage start"
            )
        self.execution.checkpoint()
        state = self.streaming.stream_state()
        self.recorder.sample(state)
        self.state = state
        return state

    def run_stream(
        self, goals: StreamGoalSource, sink, *, deadline_monotonic=None
    ) -> None:
        """连续路径的一拍。

        目标源返回 ``None`` 或队列为空时正常返回，并在返回前调 ``execution.stop()``；
        ``deadline_monotonic`` 到期抛 ``BudgetExceeded``，检查点抛 ``ExecutionError``，
        两者都由任务层决定怎么收尾。这一拍读到接触事件时，先退避并作废队列，
        然后正常返回，下一拍由任务层重新取帧补点。
        """
        self.execution.assert_owner()
        budget = (
            None if deadline_monotonic is None else ExecutionBudget(deadline_monotonic)
        )
        try:
            self._guard(budget)
        except CollisionInterrupt as interrupt:
            self._recover(interrupt.contact, clear_queue=True)
            return
        state = self.sample()
        command = goals.next(state, self.recorder.yaw)
        if command is None:
            if not self.holding:
                self.execution.stop()
                self.holding = True
            return
        if self.limiter is not None:
            command = self.limiter.limit(command, state, self.period)
        sink(command, self.period)
        self.holding = False
