"""Motion dynamics and preview boundaries, independent of model/simulator I/O."""
from dataclasses import replace
import math

import pytest

from planner.domain.motion import MotionLimits, ContinuousTrackingSettings
from planner.domain.pose import WorldPose
from planner.domain.sliding_window import QueuedWaypoint
from planner.services.motion.command_smoothing import CommandSmoother
from planner.services.motion.continuous_reference import ContinuousPathReference, distance_to_segment
from planner.services.motion.continuous_speed import angular_duration, braking_speed, stopping_distance


def test_acceleration_and_vector_jerk_remain_bounded_during_reversals_and_variable_cycles():
    smoother = CommandSmoother(1.5, 3.)
    for goal in [(2., 0., 0.), (0., 2., 0.), (-2., 0., 0.), (0., 0., -2.), (0., 0., 0.)]:
        for index in range(200):
            dt = (.025, .05, .1)[index % 3]
            acceleration = smoother.acceleration
            velocity = smoother.velocity
            output = smoother.step(goal, dt)
            assert math.hypot(*smoother.acceleration) <= 1.5+1e-9
            assert math.dist(acceleration, smoother.acceleration) <= 3*dt+1e-9
            assert math.dist(velocity, output) <= 1.5*dt+1e-9
            assert all(math.isfinite(v) for v in output)
    assert math.hypot(*smoother.velocity) < .001


def test_stop_envelope_accounts_for_jerk_current_acceleration_and_response_lag():
    assert stopping_distance(2., 1.5, 3.) > 2**2/(2*1.5)
    assert stopping_distance(2., 1.5, 3., 1., .7) > stopping_distance(2., 1.5, 3.)
    cap = braking_speed(1., 0., 2., 1.5, 3., response_time=.7)
    assert stopping_distance(cap, 1.5, 3., response_time=.7) <= 1.+1e-6
    assert braking_speed(1., 0., 2., 1.5, 3., 1., .7) < cap
    assert braking_speed(0., .5, 2., 1.5, 3.) == pytest.approx(.5, abs=1e-6)


@pytest.mark.parametrize('angle', [5., 20., 180.])
def test_yaw_timing_budget_bounds_quintic_derivatives(angle):
    limits, settings = MotionLimits(), ContinuousTrackingSettings()
    duration = angular_duration(angle, limits, settings)
    assert 1.875*angle/duration <= limits.max_yaw_rate_deg_s+1e-9
    assert 10/math.sqrt(3)*angle/duration**2 <= limits.max_yaw_acceleration_deg_s2+1e-9
    assert 60*angle/duration**3 <= settings.max_yaw_jerk_deg_s3+1e-9


def test_preview_crosses_straight_points_without_cutting_corner_or_rotation_barrier():
    path = ContinuousPathReference(ContinuousTrackingSettings(), MotionLimits())
    start, corner, end = WorldPose(0,0,0,0), WorldPose(1,0,0,0), WorldPose(1,2,0,0)
    items = (QueuedWaypoint(1,start,corner,0), QueuedWaypoint(2,corner,end,0))
    original = tuple(items)
    reference = path.sample(items, WorldPose(.8,0,0,0), 0, 2)
    assert reference.position[1] > 0
    assert distance_to_segment(reference.position, (0,0,0), (1,0,0)) <= .15+1e-6
    assert items == original
    rotation = QueuedWaypoint(2,corner,replace(corner,yaw_deg=90),90)
    reference = path.sample((items[0],rotation),WorldPose(.8,0,0,0),0,2)
    assert reference.position == (1,0,0)


def test_yaw_alignment_is_latched_when_position_drift_crosses_old_threshold():
    path = ContinuousPathReference(ContinuousTrackingSettings(), MotionLimits())
    item = QueuedWaypoint(1,WorldPose(0,0,0,0),WorldPose(1,0,0,20),20)
    assert path.sample((item,),WorldPose(.75,0,0,0),20,1).aligning
    assert path.sample((item,),WorldPose(.65,0,0,0),20,.3).aligning
    next_item = QueuedWaypoint(2,item.end,WorldPose(2,0,0,20),0)
    assert not path.sample((next_item,),WorldPose(1,0,0,20),0,.3).aligning
