import math

import pytest

from config.loader import _motion_config
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import WorldTrajectory
from planner.services.motion.synchronized import SynchronizedMotionPlanner, sample_segment


@pytest.mark.parametrize("end", [WorldPose(10, 5, -4, 90), WorldPose(0, 0, 0, -90), WorldPose(0, 0, -12, 0)])
def test_position_and_yaw_share_phase_and_respect_limits(end):
    start = WorldPose(0, 0, 0, 0)
    limits = MotionLimits()
    segment = SynchronizedMotionPlanner(2, limits).create_motion(WorldTrajectory(start, (end,))).segments[0]
    beginning = sample_segment(segment, 0)
    middle = sample_segment(segment, segment.duration_s/2)
    final = sample_segment(segment, segment.duration_s)
    assert beginning.pose == start
    assert final.pose == end
    assert beginning.velocity == final.velocity == (0, 0, 0)
    assert beginning.yaw_rate_deg_s == final.yaw_rate_deg_s == 0
    assert middle.pose.x == pytest.approx(end.x/2)
    assert middle.pose.y == pytest.approx(end.y/2)
    assert middle.pose.z == pytest.approx(end.z/2)
    assert middle.pose.yaw_deg == pytest.approx(end.yaw_deg/2)
    dt = segment.duration_s/1000
    samples = [sample_segment(segment, i*dt) for i in range(1001)]
    for a, b in zip(samples, samples[1:]):
        assert math.sqrt(sum(v*v for v in b.velocity)) <= 2 + 1e-6
        assert abs(b.yaw_rate_deg_s) <= limits.max_yaw_rate_deg_s + 1e-6
        assert math.dist(a.velocity, b.velocity)/dt <= limits.max_acceleration_mps2 + 1e-5
        assert abs(a.yaw_rate_deg_s-b.yaw_rate_deg_s)/dt <= limits.max_yaw_acceleration_deg_s2 + 1e-5


def test_shortest_yaw_crosses_wrap_without_full_turn():
    segment = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(
        WorldTrajectory(WorldPose(0, 0, 0, 170), (WorldPose(4, 0, 0, -170),))
    ).segments[0]
    midpoint = sample_segment(segment, segment.duration_s/2)
    assert abs(midpoint.pose.yaw_deg) == 180
    assert midpoint.yaw_rate_deg_s > 0
    assert wrap_yaw_deg(sample_segment(segment, segment.duration_s).pose.yaw_deg+170) == 0


@pytest.mark.parametrize("values", [{"control_hz": 0}, {"max_yaw_rate_deg_s": float("nan")},
                                   {"max_acceleration_mps2": -1}, {"enabled": "false"}, {"typo": 1}])
def test_invalid_motion_config(values):
    with pytest.raises((ValueError, RuntimeError)):
        _motion_config(values)
