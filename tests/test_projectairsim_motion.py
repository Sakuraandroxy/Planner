import math

import pytest

from planner.adapters.projectairsim.motion import SteppedMotionTracker, limit_vector
from planner.domain.execution import ExecutionBudget
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.domain.trajectory import MotionSegment
from planner.errors import BudgetExceeded, ExecutionError, MotionPaused
from planner.services.motion.synchronized import SMOOTH_PROFILE

LIMITS = MotionLimits(
    max_acceleration_mps2=5.0,
    max_yaw_rate_deg_s=90.0,
    max_yaw_acceleration_deg_s2=180.0,
    control_hz=20.0,
    position_tolerance_m=0.1,
    yaw_tolerance_deg=1.0,
    stopped_speed_mps=0.15,
    stopped_yaw_rate_deg_s=5.0,
    settle_time_s=0.1,
    min_duration_s=0.2,
)


class PerfectPlant:
    """Apply each velocity command for exactly one control period."""

    def __init__(self, pose):
        self.pose = pose
        self.commands = []

    def __call__(self, velocity, yaw_rate, measured, yaw, period):
        self.commands.append((velocity, yaw_rate, measured, yaw, period))
        self.pose = WorldPose(
            self.pose.x + velocity[0] * period,
            self.pose.y + velocity[1] * period,
            self.pose.z + velocity[2] * period,
            self.pose.yaw_deg + yaw_rate * period,
        )


class FrozenPose:
    def __init__(self, pose):
        self.pose = pose
        self.commands = []

    def __call__(self, velocity, yaw_rate, measured, yaw, period):
        self.commands.append((velocity, yaw_rate, measured, yaw))


def segment(start, end, *, speed=2.0, yaw_delta_deg=None):
    return MotionSegment(
        start=start,
        end=end,
        profile=SMOOTH_PROFILE,
        target_speed_mps=speed,
        yaw_delta_deg=yaw_delta_deg,
    )


def test_limit_vector_scales_by_magnitude():
    assert limit_vector((3.0, 4.0), 10.0) == pytest.approx((3.0, 4.0))
    assert limit_vector((3.0, 0.0), 1.5) == pytest.approx((1.5, 0.0))
    assert limit_vector((0.0, 0.0), 3.0) == pytest.approx((0.0, 0.0))
    with pytest.raises(ExecutionError):
        limit_vector((3.0, 4.0), 0.0)
    with pytest.raises(ExecutionError):
        limit_vector((float("nan"), 4.0), 1.0)


def test_tracker_drives_a_straight_segment_to_the_goal():
    plant = PerfectPlant(WorldPose(0.0, 0.0, 0.0, 0.0))
    SteppedMotionTracker(
        plant, lambda: plant.pose, LIMITS, 10.0
    ).execute(segment(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(1.0, 0.0, 0.0, 0.0)))
    assert plant.pose.x == pytest.approx(1.0, abs=0.03)
    assert plant.pose.y == pytest.approx(0.0, abs=1e-9)
    assert plant.pose.yaw_deg == pytest.approx(0.0, abs=1e-6)
    # First command is at rest on the reference start.
    assert plant.commands[0][0] == pytest.approx((0.0, 0.0, 0.0))
    # Settling runs at least the configured settle window after the profile.
    assert len(plant.commands) >= math.ceil(0.9375 * LIMITS.control_hz)


def test_tracker_reanchors_yaw_to_the_measured_heading():
    start = WorldPose(0.0, 0.0, 0.0, 10.0)
    plant = PerfectPlant(start)
    SteppedMotionTracker(
        plant, lambda: plant.pose, LIMITS, 10.0
    ).execute(
        segment(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(0.0, 0.0, 0.0, 90.0))
    )
    assert plant.pose.yaw_deg == pytest.approx(90.0, abs=1.0)


def test_tracker_requires_a_speed_limit():
    plant = PerfectPlant(WorldPose(0.0, 0.0, 0.0, 0.0))
    tracker = SteppedMotionTracker(plant, lambda: plant.pose, LIMITS, 10.0)
    with pytest.raises(ExecutionError, match="speed limit"):
        tracker.execute(
            MotionSegment(
                start=plant.pose,
                end=WorldPose(1.0, 0.0, 0.0, 0.0),
                profile=SMOOTH_PROFILE,
            )
        )
    assert plant.commands == []


def test_tracker_pauses_at_a_pose_checkpoint():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(start)
    tracker = SteppedMotionTracker(plant, lambda: plant.pose, LIMITS, 10.0)
    seen = []

    def checkpoint(pose, yaw):
        seen.append((pose, yaw))
        return pose.x > 0.4

    with pytest.raises(MotionPaused) as paused:
        tracker.execute(
            segment(start, WorldPose(1.0, 0.0, 0.0, 0.0)),
            pose_checkpoint=checkpoint,
        )
    assert paused.value.pose.x > 0.4
    assert paused.value.unwrapped_yaw_deg == pytest.approx(0.0, abs=1e-9)
    assert seen


def test_tracker_stops_when_cancelled():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(start)
    tracker = SteppedMotionTracker(
        plant, lambda: plant.pose, LIMITS, 10.0, cancelled=lambda: True
    )
    with pytest.raises(ExecutionError, match="cancelled"):
        tracker.execute(segment(start, WorldPose(1.0, 0.0, 0.0, 0.0)))
    assert plant.commands == []


def test_tracker_rejects_an_exhausted_budget_before_moving():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(start)
    tracker = SteppedMotionTracker(plant, lambda: plant.pose, LIMITS, 10.0)
    budget = ExecutionBudget(deadline_monotonic=-1.0)
    with pytest.raises(BudgetExceeded):
        tracker.execute(segment(start, WorldPose(1.0, 0.0, 0.0, 0.0)), budget=budget)
    assert plant.commands == []


def test_tracker_times_out_when_the_rover_never_moves():
    pose = WorldPose(0.0, 0.0, 0.0, 0.0)
    frozen = FrozenPose(pose)
    tracker = SteppedMotionTracker(frozen, lambda: pose, LIMITS, 1.5)
    with pytest.raises(ExecutionError, match="timed out"):
        tracker.execute(segment(pose, WorldPose(1.0, 0.0, 0.0, 0.0)))
    assert len(frozen.commands) >= math.ceil(1.5 * LIMITS.control_hz)


def test_tracker_rejects_a_duration_beyond_the_available_timeout():
    pose = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(pose)
    tracker = SteppedMotionTracker(plant, lambda: plant.pose, LIMITS, 0.5)
    with pytest.raises(ExecutionError, match="execution timeout"):
        tracker.execute(segment(pose, WorldPose(10.0, 0.0, 0.0, 0.0), speed=1.0))
    assert plant.commands == []