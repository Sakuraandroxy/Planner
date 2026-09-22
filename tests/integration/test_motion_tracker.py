"""Deterministic kinematic fake: tests tracking logic, not AirSim flight dynamics."""
import math
import sys
from types import SimpleNamespace

import pytest

from planner.adapters.airsim.motion_tracker import AirSimMotionTracker
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import WorldTrajectory
from planner.errors import ExecutionError
from planner.services.motion.synchronized import SynchronizedMotionPlanner


class SimulatedConnection:
    def __init__(self, start, moving=True, collision_after=None):
        self.pose = start
        self.time = 0.0
        self.moving = moving
        self.collision_after = collision_after
        self.commands = []
        self.hovered = False

    def sleep(self, seconds):
        self.time += seconds

    def call(self, name):
        assert name == "simGetCollisionInfo"
        return SimpleNamespace(has_collided=self.collision_after is not None and len(self.commands) >= self.collision_after)

    def call_async_and_wait(self, name, *args, **kwargs):
        if name == "hoverAsync":
            self.hovered = True
            return
        assert name == "moveByVelocityAsync"
        assert kwargs["drivetrain"] == "independent"
        assert kwargs["yaw_mode"].is_rate
        vx, vy, vz, dt = args
        rate = kwargs["yaw_mode"].yaw_or_rate
        self.commands.append((self.time, (vx, vy, vz), rate, self.pose))
        if self.moving:
            a = self.pose
            self.pose = WorldPose(a.x+vx*dt, a.y+vy*dt, a.z+vz*dt, wrap_yaw_deg(a.yaw_deg+rate*dt))
        self.time += dt


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    monkeypatch.setitem(sys.modules, "airsim", SimpleNamespace(
        DrivetrainType=SimpleNamespace(MaxDegreeOfFreedom="independent"),
        YawMode=lambda **kwargs: SimpleNamespace(**kwargs),
    ))


def run_motion(start, end, moving=True, collision_after=None, timeout=40):
    limits = MotionLimits()
    connection = SimulatedConnection(start, moving, collision_after)
    segment = SynchronizedMotionPlanner(2, limits).create_motion(WorldTrajectory(start, (end,))).segments[0]
    tracker = AirSimMotionTracker(connection, lambda: connection.pose, limits, timeout,
                                 clock=lambda: connection.time, sleep=connection.sleep)
    return connection, segment, tracker


@pytest.mark.parametrize("start,end", [
    (WorldPose(0, 0, -5, 90), WorldPose(8, 5, -8, 180)),
    (WorldPose(1, 2, -5, 170), WorldPose(1, 2, -5, -170)),
    (WorldPose(0, 0, -5, 0), WorldPose(0, 0, -17, 0)),
])
def test_reaches_position_and_yaw_with_bounded_commands(start, end):
    connection, segment, tracker = run_motion(start, end)
    tracker.execute(segment)
    final = connection.pose
    assert math.dist((final.x, final.y, final.z), (end.x, end.y, end.z)) <= 0.3
    assert abs(wrap_yaw_deg(final.yaw_deg-end.yaw_deg)) <= 2
    assert connection.hovered
    for a, b in zip(connection.commands, connection.commands[1:]):
        dt = b[0]-a[0]
        assert math.dist(a[1], b[1])/dt <= 1.5 + 1e-6
        assert abs(a[2]-b[2])/dt <= 30 + 1e-6
        assert math.sqrt(sum(v*v for v in b[1])) <= 2 + 1e-6
        assert abs(b[2]) <= 30 + 1e-6
    if end.x != start.x:
        mid = min(connection.commands, key=lambda c: abs(c[0]-segment.duration_s/2))[3]
        position_fraction = (mid.x-start.x)/(end.x-start.x)
        yaw_fraction = wrap_yaw_deg(mid.yaw_deg-start.yaw_deg)/wrap_yaw_deg(end.yaw_deg-start.yaw_deg)
        assert abs(position_fraction-yaw_fraction) < 0.06
        assert 0.4 < position_fraction < 0.6


def test_stalled_vehicle_does_not_report_success():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(3, 0, -5, 45), moving=False, timeout=10)
    with pytest.raises(ExecutionError, match="timed out"):
        tracker.execute(segment)


def test_collision_checked_during_segment():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(3, 0, -5, 45), collision_after=5)
    with pytest.raises(ExecutionError, match="collision"):
        tracker.execute(segment)
    assert len(connection.commands) == 5


def test_duration_over_timeout_rejected_before_motion():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(30, 0, -5, 90), timeout=5)
    with pytest.raises(ExecutionError, match="duration"):
        tracker.execute(segment)
    assert not connection.commands
