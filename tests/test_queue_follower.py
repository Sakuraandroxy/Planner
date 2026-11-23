"""队列跟随器：消费航点、给出期望命令，队列空就交还任务层。"""

import pytest

from planner.domain import (
    ContinuousTrackingSettings,
    HolonomicCommand,
    MotionCapability,
    MotionLimits,
    MotionModel,
    QueuedWaypoint,
    TimedVehicleState,
    WorldPose,
)
from planner.errors import ExecutionError
from planner.runtime import QueueFollower

LIMITS = MotionLimits(control_hz=20.0, position_tolerance_m=0.1, yaw_tolerance_deg=1.0)
START = WorldPose(0.0, 0.0, 0.0, 0.0)
CAPABILITY = MotionCapability(
    MotionModel.HOLONOMIC,
    max_lateral_speed_mps=2.0,
    min_turn_radius_m=None,
    allows_reverse=True,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds=1.0):
        self.now += seconds


class FakeQueue:
    """按位姿是否越过端点消耗队头；只实现跟随器用到的那部分。"""

    def __init__(self, items):
        self.items = tuple(items)
        self.reached = set()

    def consume_stream(self, measured, unwrapped_yaw, goal_yaw):
        if self.items[0].waypoint_id not in self.reached:
            return False
        self.items = self.items[1:]
        return True


class FakeExecution:
    def checkpoint(self):
        pass

    def check_deadline(self):
        pass

    def stop(self):
        raise AssertionError("the follower must not stop a queue it never drove")


class FakeStreaming:
    def __init__(self):
        self.pose = START

    def motion_capability(self):
        return CAPABILITY

    def state(self, *, timestamp_ns=1_000_000):
        return TimedVehicleState(self.pose, timestamp_ns, 0.0, (0.0, 0.0, 0.0))


def waypoint(waypoint_id, start, end, yaw_delta_deg=0.0):
    return QueuedWaypoint(waypoint_id, start, end, yaw_delta_deg)


def follower_for(queue, clock=None):
    clock = clock or FakeClock()
    follower = QueueFollower(
        FakeExecution(),
        FakeStreaming(),
        LIMITS,
        2.0,
        clock,
        60.0,
        settings=ContinuousTrackingSettings(),
    )
    follower.begin(queue)
    return follower, clock


def test_follower_consumes_reached_waypoints_and_returns_when_the_queue_empties():
    first = waypoint(1, START, WorldPose(1.0, 0.0, 0.0, 0.0))
    second = waypoint(
        2, WorldPose(1.0, 0.0, 0.0, 0.0), WorldPose(2.0, 0.0, 0.0, 0.0)
    )
    queue = FakeQueue((first, second))
    queue.reached = {1, 2}
    follower, _ = follower_for(queue)

    assert follower.next(follower.streaming.state(), 0.0) is None
    assert follower.consumed == (1, 2)
    assert queue.items == ()


def test_follower_returns_a_command_while_the_head_is_still_ahead():
    first = waypoint(1, START, WorldPose(1.0, 0.0, 0.0, 0.0))
    queue = FakeQueue((first,))
    follower, _ = follower_for(queue)

    command = follower.next(follower.streaming.state(), 0.0)

    assert isinstance(command, HolonomicCommand)
    assert follower.consumed == ()
    assert queue.items == (first,)


def test_follower_raises_when_the_head_makes_no_progress_in_time():
    first = waypoint(1, START, WorldPose(1.0, 0.0, 0.0, 0.0))
    queue = FakeQueue((first,))
    follower, clock = follower_for(queue)
    follower.next(follower.streaming.state(), 0.0)

    clock.advance(61.0)
    with pytest.raises(ExecutionError, match="waypoint timed out"):
        follower.next(follower.streaming.state(timestamp_ns=2_000_000), 0.0)
