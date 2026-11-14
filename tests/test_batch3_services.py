import math
import threading
import time

import numpy as np
import pytest

from tests.support import ue_camera_matrix
from planner.capabilities.sliding_window.scheduler import RefillScheduler
from planner.domain.execution import ExecutionBudget, ExecutionReceipt
from planner.domain.motion import MotionLimits
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.planning_window import PlanningWindowInput
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.sliding_window import SlidingWindowSettings
from planner.domain.trajectory import (
    MotionSegment,
    MotionTrajectory,
    RelativeTrajectory,
)
from planner.errors import (
    BudgetExceeded,
    ExecutionError,
    ObservationError,
    ProtocolError,
    ResourceCleanupError,
)
from planner.services.moving_observations import MovingObservations
from planner.services.planning_window import PlanningWindow
from planner.services.resource_cleanup import close_resources
from planner.services.trajectory_validation import TrajectoryValidator


def _pose(x: float = 0.0, yaw: float = 0.0) -> WorldPose:
    return WorldPose(x, 0.0, 0.0, yaw)


def _observation(*, captured: float | None = None) -> Observation:
    width, height = 8, 6
    pose = _pose()
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        depth_m=np.full((height, width), 5.0, dtype=np.float32),
        intrinsics=CameraIntrinsics.from_horizontal_fov(width, height, 90.0),
        camera_matrix=ue_camera_matrix(pose.x, pose.y, -pose.z, pose.yaw_deg),
        timestamp_ns=10,
        captured_monotonic=time.monotonic() if captured is None else captured,
        capture_id="capture-1",
        stopped=True,
    )


class _Planner:
    def __init__(self, points):
        self.points = points
        self.calls = []

    def plan(self, observation, instruction, **context):
        self.calls.append((observation, instruction, context))
        return RelativeTrajectory(tuple(self.points))


def _window(points=None):
    planner = _Planner(points or (RelativePoseDelta(0.1, 0.0, 0.0, 0.0),) * 5)
    window = PlanningWindow(
        planner,
        TrajectoryValidator(5, 2.0, 45.0),
        SlidingWindowSettings(),
        MotionLimits(),
        time.monotonic,
    )
    return window, planner


def _planning(pose=None) -> PlanningWindowInput:
    return PlanningWindowInput(pose or _pose(), 30.0, 10.0)


def test_planning_window_begin_validates_deadline_and_state():
    window, _ = _window()
    now = time.monotonic()
    with pytest.raises(BudgetExceeded):
        window.begin(deadline_monotonic=now - 1.0, maximum_planning_rounds=2)
    with pytest.raises(ValueError):
        window.begin(deadline_monotonic=float("nan"), maximum_planning_rounds=2)
    with pytest.raises(ValueError):
        window.begin(deadline_monotonic=now + 60.0, maximum_planning_rounds=0)
    with pytest.raises(ValueError):
        window.begin(deadline_monotonic=None, maximum_planning_rounds=0)
    window.begin(deadline_monotonic=now + 60.0, maximum_planning_rounds=2)
    with pytest.raises(ExecutionError):
        window.begin(deadline_monotonic=now + 60.0, maximum_planning_rounds=2)
    window.end()
    window.close()


def test_planning_window_accepts_an_unbounded_stage():
    window, planner = _window()
    window.begin(deadline_monotonic=None, maximum_planning_rounds=None)
    assert window.active is True
    assert math.isinf(window.deadline)
    assert window.background.request_limit is None
    for _ in range(3):
        world = window.next_world(_observation(), None, "drive", _planning(),
                                  lambda: None, lambda: None)
        assert len(world.poses) == 5
        window.queue.clear()
    assert len(planner.calls) == 3
    window.end()
    window.close()


def test_planning_window_loads_and_compares_approved_prefix():
    window, _ = _window()
    window.begin(deadline_monotonic=time.monotonic() + 60.0, maximum_planning_rounds=2)
    segment = MotionSegment(_pose(0.0), _pose(1.0), "linear", duration_s=1.0, yaw_delta_deg=0.0)
    motion = MotionTrajectory((segment,))
    window.load(motion)
    assert len(window.queue.items) == 1
    assert window.stream_due() is True
    with pytest.raises(ProtocolError):
        window.load(MotionTrajectory((MotionSegment(_pose(0.0), _pose(2.0), "linear", yaw_delta_deg=0.0),)))
    window.close()


def test_planning_window_next_world_plans_and_anchors_five_points():
    window, planner = _window()
    window.begin(deadline_monotonic=time.monotonic() + 60.0, maximum_planning_rounds=2)
    observation = _observation()
    requested = []
    world = window.next_world(observation, None, "drive", _planning(), lambda: requested.append(1),
                              lambda: None)
    assert len(planner.calls) == 1
    assert requested == [1]
    assert len(world.poses) == 5
    assert world.poses[-1].x == pytest.approx(0.5)
    window.close()


def test_planning_window_next_world_rejects_unfinished_queue():
    window, _ = _window()
    window.begin(deadline_monotonic=time.monotonic() + 60.0, maximum_planning_rounds=2)
    window.load(MotionTrajectory((MotionSegment(_pose(0.0), _pose(1.0), "linear", yaw_delta_deg=0.0),)))
    with pytest.raises(ProtocolError):
        window.next_world(_observation(), None, "drive", _planning(), lambda: None, lambda: None)
    window.close()


def test_planning_window_executed_consumes_and_records():
    window, _ = _window()
    window.begin(deadline_monotonic=time.monotonic() + 60.0, maximum_planning_rounds=2)
    start, end = _pose(0.0), _pose(1.0)
    window.load(MotionTrajectory((MotionSegment(start, end, "linear", yaw_delta_deg=0.0),)))
    receipt = ExecutionReceipt(1, end, 1.0, measured_actions=(RelativePoseDelta(1.0, 0.0, 0.0, 0.0),))
    window.executed(start, receipt, MotionTrajectory(()))
    assert window.queue.items == ()
    assert window.record.translation_m == pytest.approx(1.0)
    window.close()


def test_refill_scheduler_uses_latency_margin():
    scheduler = RefillScheduler(pending_threshold=2, margin_s=1.0)
    assert scheduler.should_refill(0, 100.0) is False
    assert scheduler.should_refill(1, 100.0) is True
    assert scheduler.should_refill(5, 100.0) is False
    assert scheduler.should_refill(5, 1.0) is True
    scheduler.record_latency(2.0)
    assert scheduler.should_refill(5, 6.0) is False


class _MovingSource:
    def __init__(self, value=None, error=None):
        self.value, self.error = value, error
        self.closed = False
        self.calls = 0

    def capture_moving(self, *, budget=None):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return self.value

    def close(self):
        self.closed = True


def _wait_for(predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        threading.Event().wait(0.01)
    return False


def test_moving_observations_returns_frames_and_closes_source():
    observation = _observation()
    source = _MovingSource(observation)
    frames = MovingObservations(source, time.monotonic, timeout_s=2.0)
    frames.begin()
    budget = ExecutionBudget(time.monotonic() + 5.0)
    frames.request(budget)
    assert _wait_for(lambda: frames.poll() is not None)
    assert frames.latest is observation
    frames.close()
    assert source.closed is True


def test_moving_observations_fails_closed_after_the_failure_budget():
    frames = MovingObservations(_MovingSource(error=RuntimeError("camera")), time.monotonic,
                                timeout_s=2.0, maximum_failures=1)
    frames.begin()
    frames.request(ExecutionBudget(time.monotonic() + 5.0))
    with pytest.raises(ObservationError):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            frames.poll()


def test_close_resources_attempts_every_callback_and_aggregates():
    calls = []
    close_resources((lambda: calls.append(1), lambda: calls.append(2)))
    assert calls == [1, 2]
    calls.clear()

    def bad(name, exc):
        def close():
            calls.append(name)
            raise exc
        return close

    with pytest.raises(ResourceCleanupError) as info:
        close_resources((bad("a", RuntimeError("camera")), lambda: calls.append("ok"),
                         bad("b", ValueError("actor"))))
    assert calls == ["a", "ok", "b"]
    assert len(info.value.failures) == 2
    assert "RuntimeError" in str(info.value) and "ValueError" in str(info.value)


def test_close_resources_propagates_interrupts():
    def interrupted():
        raise KeyboardInterrupt()

    with pytest.raises(KeyboardInterrupt):
        close_resources((interrupted,))