"""连续路径的公共编排：一拍之内的顺序、停车与暂停。"""

from types import SimpleNamespace

import pytest

from planner.domain import WorldPose
from planner.workflows.common.streaming import StreamPolicy, StreamRunner

START = WorldPose(0.0, 0.0, 0.0, 0.0)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeLoop:
    """只记调用顺序；``stop_after`` 拍之后抛 ``KeyboardInterrupt`` 收尾。"""

    period = 0.05

    def __init__(self, log, *, stop_after=None):
        self.log = log
        self.state = SimpleNamespace(pose=START)
        self.recorder = SimpleNamespace(record=object())
        self.deadlines = []
        self.ticks = 0
        self.stop_after = stop_after

    def stream_sink(self):
        return "sink"

    def run_stream(self, goals, sink, *, deadline_monotonic=None):
        self.ticks += 1
        if self.stop_after is not None and self.ticks > self.stop_after:
            raise KeyboardInterrupt
        self.log.append("drive")
        self.deadlines.append(deadline_monotonic)


class FakeQueue:
    def __init__(self, items):
        self.items = items


class FakeWindow:
    def __init__(self, log, *, due=True, items=(), authorized=("planning_submitted", 1, "ok")):
        self.log = log
        self.queue = FakeQueue(items)
        self.due = due
        self.authorized = authorized
        self.record = None

    def stream_due(self):
        return self.due

    def poll_stream(self, measured, checkpoint):
        self.log.append("poll_stream")
        return self.authorized


class FakeFrames:
    def __init__(self, log, *, frame=None):
        self.log = log
        self.job = SimpleNamespace(future=None, busy=False)
        self.frame = frame
        self.requests = []

    def poll(self):
        self.log.append("poll")
        frame, self.frame = self.frame, None
        return frame

    def request(self, budget):
        self.log.append("request")
        self.requests.append(budget)


class FakeExecution:
    def __init__(self, log):
        self.log = log

    def checkpoint(self):
        self.log.append("checkpoint")

    def stop(self):
        self.log.append("stop")


def _observation(capture_id):
    return SimpleNamespace(capture_id=capture_id)


def _runner(
    log,
    *,
    clock=None,
    stop_after=None,
    due=True,
    items=(),
    frame=None,
    pause_when=None,
    policy=StreamPolicy(),
    deadline_monotonic=None,
):
    loop = FakeLoop(log, stop_after=stop_after)
    frames = FakeFrames(log, frame=frame)
    window = FakeWindow(log, due=due, items=items)
    runner = StreamRunner(
        loop,
        "follower",
        frames,
        FakeExecution(log),
        clock=clock or FakeClock(),
        sleep=lambda seconds: None,
        capture_timeout_s=2.0,
        pause_when=pause_when,
        deadline_monotonic=deadline_monotonic,
        policy=policy,
    )
    return runner, loop, frames, window


def _drive_until_interrupted(runner, window, observation):
    with pytest.raises(KeyboardInterrupt):
        runner.run(window, observation)


def test_a_tick_polls_a_frame_before_authorizing_the_refill():
    log = []
    policy = StreamPolicy(
        authorized=lambda result, window: log.append("authorized"),
        submit=lambda observation, window: log.append("submit"),
    )
    runner, loop, _, window = _runner(
        log, stop_after=1, frame=_observation("moving-1"), policy=policy
    )

    _drive_until_interrupted(runner, window, _observation("initial"))

    assert log == [
        "drive",
        "poll",
        "poll_stream",
        "authorized",
        "submit",
        "request",
    ]
    assert loop.deadlines == [None]
    assert window.record is loop.recorder.record


def test_a_refill_waits_for_a_frame_it_has_not_used_yet():
    log = []
    policy = StreamPolicy(submit=lambda observation, window: log.append("submit"))
    runner, _, _, window = _runner(
        log, stop_after=2, frame=_observation("moving-1"), policy=policy
    )

    _drive_until_interrupted(runner, window, _observation("initial"))

    # 第二拍没有新帧，同一个 capture_id 不再补点。
    assert log.count("poll") == 2
    assert log.count("submit") == 1
    assert log.count("request") == 2


def test_an_empty_queue_stops_once_per_tick():
    log = []
    policy = StreamPolicy(idle=lambda window: log.append("idle"))
    runner, _, _, window = _runner(log, stop_after=2, items=(), policy=policy)

    _drive_until_interrupted(runner, window, _observation("initial"))

    # 队列一直为空，每一拍停一次；停稳状态由执行层保持，任务层不重复。
    assert log.count("idle") == log.count("drive") == 2


def test_a_non_empty_queue_never_stops_the_vehicle():
    log = []
    policy = StreamPolicy(idle=lambda window: log.append("idle"))
    runner, _, _, window = _runner(
        log, stop_after=1, items=(object(),), policy=policy
    )

    _drive_until_interrupted(runner, window, _observation("initial"))

    assert "idle" not in log


def test_a_paused_tick_fetches_no_frames_and_drives_nothing():
    log = []
    policy = StreamPolicy(
        authorized=lambda result, window: log.append("authorized"),
        submit=lambda observation, window: log.append("submit"),
    )
    runner, loop, _, window = _runner(
        log,
        pause_when=lambda pose: "inside pause radius",
        policy=policy,
    )

    reason = runner.tick(window)

    assert reason == "inside pause radius"
    assert log == ["stop"]
    assert loop.ticks == 0


def test_a_capture_request_is_single_flight_and_bounded_by_the_timeout():
    log = []
    runner, loop, frames, window = _runner(
        log, stop_after=1, deadline_monotonic=3.5
    )
    frames.job.busy = True

    _drive_until_interrupted(runner, window, _observation("initial"))

    # 上一帧还在跑：这一拍不重复请求。
    assert frames.requests == []
    assert loop.deadlines == [3.5]

    frames.job.busy = False
    clock = FakeClock()
    runner, _, frames, window = _runner(log, stop_after=1, clock=clock)

    _drive_until_interrupted(runner, window, _observation("initial"))

    assert len(frames.requests) == 1
    assert frames.requests[0].remaining(clock()) == pytest.approx(2.0)
