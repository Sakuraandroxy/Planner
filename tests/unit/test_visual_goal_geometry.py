from dataclasses import replace
import math

import numpy as np
import pytest

from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.targeting.association import make_reference
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.domain.pose import WorldPose
from planner.domain.visual_goal import VisualGoalNavigationParameters
from planner.domain.visual_goal_limits import VisualGoalLimits
from tests.fakes.visual_goal import wall_observation, VISIBLE


@pytest.mark.parametrize("depth_size", [(128, 128), (256, 144), (64, 64)])
def test_ray_range_mapping_handles_different_resolution_and_aspect(depth_size):
    observation = wall_observation(4, size=(192, 108), depth_size=depth_size)
    estimate = DepthGeometry(0.5, 200, True).estimate(observation, VISIBLE)
    assert estimate.reliable
    assert estimate.distance_m == pytest.approx(4, abs=0.01)
    assert estimate.world_point == pytest.approx((4, 0, -4), abs=0.01)


def test_ray_range_is_not_optical_axis_depth_at_image_edge():
    observation = wall_observation(4)
    goal = replace(VISIBLE, bbox=(0.5, 0.2, 0.95, 0.8), surface_point=(0.8, 0.5))
    estimate = DepthGeometry(0.5, 200, True).estimate(observation, goal)
    assert estimate.reliable
    assert estimate.world_point[0] == pytest.approx(4, abs=0.03)
    assert estimate.world_point[1] == pytest.approx(2.4, abs=0.03)
    assert estimate.distance_m == pytest.approx(math.hypot(4, 2.4), abs=0.03)


def test_camera_world_rotation_and_offset_are_used():
    obs = wall_observation(4, WorldPose(2, 3, -5, 90))
    obs = replace(obs, camera_position_world=(2, 3.2, -5))
    estimate = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE)
    assert estimate.world_point == pytest.approx((2, 7.2, -5), abs=0.01)
    assert estimate.distance_m == pytest.approx(4.2, abs=0.01)


@pytest.mark.parametrize("kind", ["unknown", "optical_depth"])
def test_unverified_depth_semantics_fail_closed(kind):
    obs = replace(wall_observation(), depth_kind=kind)
    assert not DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE).reliable


def test_invalid_depth_is_rejected_but_mixed_neighbors_do_not_affect_the_pixel():
    obs = wall_observation()
    limits = VisualGoalLimits()
    obs.depth_meters[:] = np.nan
    assert not DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE).reliable
    obs.depth_meters[:] = 4
    # Only the selected pixel is used even when surrounding depths vary.
    ys, xs = np.mgrid[:5, :5]
    obs.depth_meters[62:67, 62:67] = np.where((ys+xs) % 2, 8, 4)
    result = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE)
    assert result.reliable and result.distance_m == 4




@pytest.mark.parametrize("distance,candidate", [(3, True), (3.5, True), (3.8, False), (2, True), (4.9, False), (8, False)])
def test_arrival_uses_radius_upper_bound_without_clearance_lower_bound(distance, candidate):
    obs = wall_observation(distance)
    limits = VisualGoalLimits()
    estimate = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE)
    check = VisualGoalCompletion(limits)
    params = VisualGoalNavigationParameters("approach red house", "red house")
    assert check.candidate(params, obs, VISIBLE, estimate, obs.vehicle_pose, radius_m=4) is candidate
    assert not check.candidate(params, replace(obs, stopped=False), VISIBLE, estimate, obs.vehicle_pose, radius_m=4)
