import threading
from dataclasses import replace

import pytest

from planner.domain import ContinuousTrackingSettings, MotionLimits
from planner.domain import WorldPose
from planner.domain import TimedVehicleState
from planner.errors import ExecutionError
from planner.runtime.position_hold import PositionHold

LIMITS = MotionLimits(
    max_acceleration_mps2=1.5,
    max_yaw_rate_deg_s=30.0,
    max_yaw_acceleration_deg_s2=60.0,
    control_hz=20.0,
    position_tolerance_m=0.05,
    yaw_tolerance_deg=1.0,
    stopped_speed_mps=0.15,
    stopped_yaw_rate_deg_s=2.0,
    settle_time_s=0.1,
    min_duration_s=0.2,
    heading_hold_gain=2.0,
    heading_rate_damping=0.6,
    heading_deadband_deg=0.5,
    heading_rate_deadband_dps=1.5,
)
TRACKING = ContinuousTrackingSettings()


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))


class HoldVehicle:
    """Apply each hold command for its duration and advance the sim clock."""

    def __init__(self, pose, clock, *, apply=True, yaw_rate_deg_s=0.0):
        self.pose = pose
        self.clock = clock
        self.apply = apply
        self.timestamp_ns = 1_000_000
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw_rate_deg_s = yaw_rate_deg_s
        self.commands = []
        self.cancels = 0

    def stream_state(self):
        return TimedVehicleState(
            self.pose, self.timestamp_ns, self.clock(), self.velocity,
            yaw_rate_deg_s=self.yaw_rate_deg_s,
        )

    def cancel(self):
        self.cancels += 1

    def hold_velocity(self, velocity, yaw_rate_deg_s, duration_s):
        self.commands.append((velocity, yaw_rate_deg_s, duration_s))
        if not self.apply:
            return
        self.pose = WorldPose(
            self.pose.x + velocity[0] * duration_s,
            self.pose.y + velocity[1] * duration_s,
            self.pose.z + velocity[2] * duration_s,
            self.pose.yaw_deg + yaw_rate_deg_s * duration_s,
        )
        self.velocity = velocity
        self.timestamp_ns += round(duration_s * 1e9)


def build(vehicle, clock, timeout_s=5.0):
    return PositionHold(
        vehicle,
        LIMITS,
        TRACKING,
        2.0,
        timeout_s,
        clock=clock,
        sleep=clock.sleep,
    )


def test_start_captures_the_reference_and_cancels_pending_motion():
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(3.0, 4.0, 5.0, 30.0), clock)
    hold = build(vehicle, clock)
    hold.start()
    assert hold.target == WorldPose(3.0, 4.0, 5.0, 30.0)
    assert hold.active is True
    assert vehicle.cancels == 1


def test_wait_stabilizes_a_stepped_vehicle_and_commands_one_period():
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(0.0, 0.0, -3.0, 0.0), clock)
    hold = build(vehicle, clock)
    hold.start()
    hold.wait()
    assert hold.observable is True
    assert vehicle.commands
    assert all(
        duration == pytest.approx(1 / LIMITS.control_hz)
        for _, _, duration in vehicle.commands
    )
    # The first tick sees the unchanged timestamp, commands one period and
    # only then joins the observable window.
    assert vehicle.timestamp_ns > 1_000_000


def test_wait_times_out_when_the_simulator_never_advances():
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(0.0, 0.0, -3.0, 0.0), clock, apply=False)
    hold = build(vehicle, clock, timeout_s=0.5)
    hold.start()
    with pytest.raises(ExecutionError, match="did not stabilize"):
        hold.wait()
    assert vehicle.commands


def test_tick_rejects_a_foreign_thread():
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(0.0, 0.0, -3.0, 0.0), clock)
    hold = build(vehicle, clock)
    hold.start()
    errors = []

    def foreign():
        try:
            hold.tick()
        except ExecutionError as exc:
            errors.append(exc)

    thread = threading.Thread(target=foreign)
    thread.start()
    thread.join()
    assert errors and "control owner" in str(errors[0])
    assert vehicle.commands == []


def test_contains_pose_uses_the_reference_bounds():
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(0.0, 0.0, -3.0, 0.0), clock)
    hold = build(vehicle, clock)
    hold.start()
    assert hold.contains_pose(WorldPose(0.01, 0.01, -3.01, 0.5))
    assert not hold.contains_pose(WorldPose(1.0, 0.0, -3.0, 0.0))


def test_approach_target_reserves_braking_distance():
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(0.0, 0.0, 0.0, 0.0), clock)
    hold = build(vehicle, clock)
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    assert hold.approach_target(start, WorldPose(0.01, 0.0, 0.0, 0.0)) is None
    handoff = hold.approach_target(start, WorldPose(100.0, 0.0, 0.0, 0.0))
    assert handoff is not None
    assert 0.0 < handoff.x < 100.0

def _settled_yaw_command(yaw_rate_deg_s, ticks=200):
    """Yaw command after the smoother settles, with the pose held still."""
    clock = FakeClock()
    vehicle = HoldVehicle(
        WorldPose(0.0, 0.0, -3.0, 10.0), clock, apply=False, yaw_rate_deg_s=yaw_rate_deg_s
    )
    hold = build(vehicle, clock)
    # 目标固定比当前朝向多 10°；车身不动，误差因此一直存在。
    hold.start(WorldPose(0.0, 0.0, -3.0, 20.0))
    for _ in range(ticks):
        clock.sleep(1 / LIMITS.control_hz)
        hold.tick()
    return vehicle.commands[-1][1]


def test_hold_damps_the_yaw_command_with_the_measured_rate():
    still = _settled_yaw_command(0.0)
    turning = _settled_yaw_command(5.0)
    assert still > 10.0
    assert turning == pytest.approx(still - 0.6 * 5.0, abs=0.1)


def test_hold_stops_correcting_inside_the_heading_deadband():
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(0.0, 0.0, -3.0, 0.2), clock)
    hold = build(vehicle, clock)
    hold.start()
    hold.tick()
    assert vehicle.commands[0][1] == 0.0


def test_hold_uses_its_own_gain_not_the_flight_path_gain():
    """The hold faces a fixed angle, so its gain is tuned apart from ``yaw_gain``."""
    clock = FakeClock()
    vehicle = HoldVehicle(WorldPose(0.0, 0.0, -3.0, 10.0), clock, apply=False)
    limits = replace(LIMITS, yaw_gain=2.0, heading_hold_gain=0.4, heading_rate_damping=0.0)
    hold = PositionHold(vehicle, limits, TRACKING, 2.0, 5.0, clock=clock, sleep=clock.sleep)
    hold.start(WorldPose(0.0, 0.0, -3.0, 20.0))
    for _ in range(200):
        clock.sleep(1 / limits.control_hz)
        hold.tick()
    assert vehicle.commands[-1][1] == pytest.approx(0.4 * 10.0, abs=0.1)
