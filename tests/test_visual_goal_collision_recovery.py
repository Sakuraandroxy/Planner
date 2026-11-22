"""Visual-goal collision recovery: contact -> reverse -> clear queue -> replan.

Mirrors the free-navigation recovery that previously existed only there. The
"drive to the white car" task used to ram the target because visual-goal
navigation never backed off; this pins the new behaviour.
"""

from types import SimpleNamespace

import pytest

from planner.domain import WorldPose
from planner.errors import ExecutionError
from planner.workflows.visual_goal_navigation import VisualGoalContinuous


class _Follower:
    def __init__(self):
        self.reset_called = False

    def reset(self, *args, **kwargs):
        self.reset_called = True


class _Stream:
    def __init__(self):
        self.state = SimpleNamespace(pose=WorldPose(0.0, 0.0, 0.0, 0.0))
        self.goal_yaw, self.head_id, self.paused, self.holding = 1.0, 7, "x", False
        self.follower = _Follower()
        self.initial, self.actions, self.record = None, [1, 2], object()
        self.path_length_m = 5.0
        self.sample_calls = 0

    def sample(self):
        self.sample_calls += 1


class _Queue:
    def __init__(self):
        self.items = ("a", "b")
        self.cleared = False

    def clear(self):
        self.cleared = True
        self.items = ()


class _Background:
    def __init__(self):
        self.discarded = False

    def discard(self):
        self.discarded = True


class _Window:
    def __init__(self):
        self.background = _Background()
        self.queue = _Queue()


class _Job:
    busy = False

    def discard(self):
        pass


class _Frames:
    def __init__(self):
        self.job = _Job()
        self.failures, self.retry_at = 3, 9.0

    def discard(self):
        self.job.discard()


class _Targets:
    def __init__(self):
        self.job = _Job()


class _Vehicle:
    def __init__(self):
        self.last_collision = SimpleNamespace(object_name="WhiteCar")

    def collision_state(self):
        return False


class _Execution:
    def __init__(self):
        self.vehicle = _Vehicle()
        self.stopped = False
        self.executed = []

    def stop(self):
        self.stopped = True

    def execute(self, trajectory):
        self.executed.append(trajectory)


class _Motion:
    def create_motion(self, world):
        return ("motion", world)


def _follower_with(**overrides):
    follower = object.__new__(VisualGoalContinuous)
    follower.execution = _Execution()
    follower.stream = _Stream()
    follower.frames = _Frames()
    follower.targets = _Targets()
    follower.motion = _Motion()
    follower.maximum_collision_recoveries = 3
    follower.collision_recoveries = 0
    follower.collision_backoff_m = 3.0
    follower.collision_recovery_reset_m = 20.0
    follower.collision_reset_baseline_m = None
    follower.clock = lambda: 0.0
    follower.sleep = lambda seconds: pytest.fail("recovery should not wait")
    follower.limits = SimpleNamespace(model_request_timeout_s=1.0)
    follower.capture_stopped = lambda session: SimpleNamespace(fresh=True)
    for name, value in overrides.items():
        setattr(follower, name, value)
    return follower


def _session():
    return SimpleNamespace(window=_Window())


def test_recovery_reverses_clears_queue_and_returns_a_fresh_observation():
    follower = _follower_with()
    session = _session()
    result = follower._recover_from_collision(session)

    assert follower.execution.stopped is True          # stop before reversing
    assert follower.execution.executed               # a backoff motion was executed
    assert session.window.queue.cleared is True      # remaining waypoints dropped
    assert session.window.background.discarded is True
    assert follower.frames.failures == 0 and follower.frames.retry_at == 0.0
    assert follower.stream.follower.reset_called is True
    assert follower.stream.sample_calls >= 1
    assert follower.collision_recoveries == 1
    assert result.fresh is True                       # replanning gets a new frame


def test_recovery_backs_off_farther_each_time():
    follower = _follower_with(collision_recoveries=1)
    follower._recover_from_collision(_session())
    assert follower.collision_recoveries == 2


def test_recovery_budget_is_bounded():
    follower = _follower_with(collision_recoveries=3)
    with pytest.raises(ExecutionError, match="budget exhausted"):
        follower._recover_from_collision(_session())


def test_recovery_requires_a_motion_planner():
    follower = _follower_with(motion=None)
    with pytest.raises(ExecutionError, match="requires a motion planner"):
        follower._recover_from_collision(_session())
