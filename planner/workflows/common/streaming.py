"""连续路径一拍之内的公共编排：驱动、取帧、轮询规划窗、补点与节拍。

任务包只提供判定（停滞、测量、补点）；sink 与 deadline 怎么交给
``ControlLoop.run_stream``、移动帧的取帧与单飞门控、队列空时停车、节拍补齐
都在这里。碰撞退避由执行层的 ``CollisionRecovery`` 处理，本模块不读碰撞端口。
视觉目标任务的连续路径要等暂停与确认的接入口定了再接进来。
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from planner.domain import ExecutionBudget


@dataclass(frozen=True)
class StreamPolicy:
    """一拍之内的任务判定；留 ``None`` 的步骤整段跳过。

    每个回调都拿到本阶段的规划窗，只用到 ``queue``、``poll_stream``、
    ``stream_due`` 与 ``record`` 四项。
    """

    #: ``poll_stream`` 授权了新一批航点，参数是窗口的原始返回值。
    authorized: Callable[[tuple, object], None] | None = None
    #: 停滞判定，超时由任务包抛自己的失败类型。
    progress: Callable[[object], None] | None = None
    #: 队列空时停车一次，每一拍调用，停稳状态由执行层自己保持。
    idle: Callable[[object], None] | None = None
    #: 记录测量与发布遥测。
    measure: Callable[[object], None] | None = None
    #: 补点，返回窗口是否接受。
    submit: Callable[[object, object], bool | None] | None = None


class StreamRunner:
    """连续路径的公共编排：两次 ``run_stream`` 之间该做的事都收在这里。

    一拍的动作固定为：暂停判定 → 驱动 → 交接执行记录 → 取移动帧 →
    ``window.poll_stream`` → 停滞判定 → 队列空停车 → 测量遥测 → 补点 →
    请求下一次取帧 → 补齐节拍。
    """

    def __init__(
        self,
        loop,
        follower,
        frames,
        execution,
        *,
        clock,
        sleep,
        capture_timeout_s: float,
        pause_when: Callable[[object], str | None] | None = None,
        deadline_monotonic: float | None = None,
        policy: StreamPolicy = StreamPolicy(),
    ):
        if pause_when is not None and not callable(pause_when):
            raise ValueError("stream pause predicate must be callable")
        self.loop, self.follower, self.frames = loop, follower, frames
        self.execution = execution
        self.clock, self.sleep = clock, sleep
        self.capture_timeout_s = float(capture_timeout_s)
        self.pause_when = pause_when
        self.deadline_monotonic = deadline_monotonic
        self.policy = policy
        self.observation = None
        self.measured = None
        self.used_capture = None

    def run(self, window, observation) -> None:
        """跑到一次驱动抛错为止；停止与预算耗尽都由 ``ControlLoop`` 抛出。"""
        self.observation = observation
        self.used_capture = None
        while True:
            self.tick(window)

    def tick(self, window) -> str | None:
        """走一拍；返回暂停理由，返回 ``None`` 表示这一拍照常驱动。"""
        started = self.clock()
        reason = self._pause_reason()
        if reason is not None:
            # 暂停之后等确认结果，这一拍既不驱动也不再取移动帧。
            self.execution.stop()
            self._pace(started)
            return reason
        self.loop.run_stream(
            self.follower,
            self.loop.stream_sink(),
            deadline_monotonic=self.deadline_monotonic,
        )
        self.measured = self.loop.state
        window.record = self.loop.recorder.record
        frame = self.frames.poll()
        if frame is not None:
            self.observation = frame
        authorized = window.poll_stream(
            self.measured.pose, self.execution.checkpoint
        )
        if authorized is not None and self.policy.authorized is not None:
            self.policy.authorized(authorized, window)
        if self.policy.progress is not None:
            self.policy.progress(window)
        if not window.queue.items and self.policy.idle is not None:
            self.policy.idle(window)
        if self.policy.measure is not None:
            self.policy.measure(window)
        due = window.stream_due()
        if due and self.used_capture != self.observation.capture_id:
            self.used_capture = self.observation.capture_id
            if self.policy.submit is not None:
                self.policy.submit(self.observation, window)
        if due and self._capture_idle():
            self.frames.request(
                ExecutionBudget(self.clock() + self.capture_timeout_s)
            )
        self._pace(started)
        return None

    def _pause_reason(self) -> str | None:
        if self.pause_when is None:
            return None
        measured = self.loop.state
        return self.pause_when(None if measured is None else measured.pose)

    def _capture_idle(self) -> bool:
        job = self.frames.job
        return job.future is None and not job.busy

    def _pace(self, started: float) -> None:
        self.sleep(max(0.001, self.loop.period - (self.clock() - started)))
