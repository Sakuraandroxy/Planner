import math

import pytest

from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.services.segment_motion import (
    MotionLimits,
    duration_for,
    plan_segment,
    quintic,
)


def test_quintic_boundaries_and_midpoint():
    assert quintic(0.0) == pytest.approx(0.0)
    assert quintic(0.5) == pytest.approx(0.5)
    assert quintic(1.0) == pytest.approx(1.0)


def test_quintic_is_monotonic():
    values = [quintic(i / 50.0) for i in range(51)]
    assert all(b >= a for a, b in zip(values, values[1:]))


def test_duration_respects_minimum():
    limits = MotionLimits(min_duration_s=1.0, max_speed_mps=3.0, max_accel_mps2=1.5)
    duration = duration_for(RelativePoseDelta(dx=0.01), limits)
    assert duration >= 1.0


def test_plan_segment_endpoints():
    limits = MotionLimits(min_duration_s=0.5)
    start = WorldPose(1.0, 2.0, -3.0, 10.0)
    setpoints = plan_segment(start, RelativePoseDelta(dx=1.0, dy=2.0, dz=-1.0), limits)
    assert setpoints[0].pose == start
    expected = start.offset(1.0, 2.0, -1.0)
    assert setpoints[-1].pose.x == pytest.approx(expected.x)
    assert setpoints[-1].pose.y == pytest.approx(expected.y)
    assert setpoints[-1].pose.z == pytest.approx(expected.z)


def test_plan_segment_yaw_wraps():
    limits = MotionLimits(min_duration_s=0.5)
    start = WorldPose(0.0, 0.0, 0.0, 170.0)
    setpoints = plan_segment(start, RelativePoseDelta(dyaw_deg=30.0), limits)
    assert setpoints[-1].pose.yaw_deg == pytest.approx(-160.0)


def test_plan_segment_step_count():
    limits = MotionLimits(control_hz=20.0, min_duration_s=1.0)
    setpoints = plan_segment(WorldPose(0, 0, 0, 0), RelativePoseDelta(dx=0.1), limits)
    duration = setpoints[-1].t
    assert len(setpoints) == max(2, math.ceil(duration * 20.0) + 1)


def test_delta_without_vertical_drops_dz():
    delta = RelativePoseDelta(dx=1.0, dy=2.0, dz=-3.0, dyaw_deg=4.0)
    flat = delta.without_vertical()
    assert (flat.dx, flat.dy, flat.dz, flat.dyaw_deg) == (1.0, 2.0, 0.0, 4.0)


def test_pose_at_height_clamps_z():
    pose = WorldPose(3.0, 4.0, -5.0, 90.0)
    clamped = pose.at_height(0.25)
    assert clamped == WorldPose(3.0, 4.0, 0.25, 90.0)