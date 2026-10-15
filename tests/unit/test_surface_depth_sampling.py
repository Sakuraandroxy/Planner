"""Single-pixel depth and fixed-reference regressions, entirely offline."""
from dataclasses import replace
from unittest.mock import Mock

import numpy as np
import pytest

from planner.adapters.target_detection.client import parse_target_detection
from planner.capabilities.targeting.association import make_reference
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.domain.visual_goal import GoalVisibility
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.pose import WorldPose
from planner.adapters.airsim.camera_model import intrinsics_from_horizontal_fov
from tests.fakes.target_detection import detection_json
from tests.fakes.visual_goal import VISIBLE, wall_observation


def test_exact_pixel_is_used_even_when_all_neighbors_disagree():
    obs = wall_observation(8)
    obs.depth_meters[:] = 8
    obs.depth_meters[64, 64] = 4.123456789
    result = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE)
    assert result.reliable and result.distance_m == pytest.approx(4.123456789)
    assert "mode=direct_pixel" in result.reason and "pixel=[64,64]" in result.reason


@pytest.mark.parametrize("value", [3, 8])
def test_minority_pixel_is_never_replaced_by_majority_neighbor_depth(value):
    obs = wall_observation(6)
    obs.depth_meters[:] = 6
    obs.depth_meters[64, 64] = value
    result = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE)
    assert result.reliable and result.distance_m == value


@pytest.mark.parametrize("invalid", [np.nan, np.inf, 0, -1, .1, 201])
def test_invalid_pixel_cannot_be_filled_with_valid_neighbors(invalid):
    obs = wall_observation(4)
    obs.depth_meters[64, 64] = invalid
    result = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE)
    assert not result.reliable and "depth is invalid" in result.reason


def test_readonly_raw_depth_is_unchanged():
    obs = wall_observation(4)
    obs.depth_meters[62, 62] = np.nan
    before = obs.depth_meters.copy()
    obs.depth_meters.setflags(write=False)
    result = DepthGeometry(0.5, 200, True).estimate(obs, VISIBLE)
    assert result.reliable and np.array_equal(before, obs.depth_meters, equal_nan=True)
    assert not obs.depth_meters.flags.writeable


def test_tiny_bbox_needs_only_its_mapped_pixel():
    obs = wall_observation(4)
    goal = replace(VISIBLE, bbox=(.499, .499, .501, .501))
    result = DepthGeometry(0.5, 200, True).estimate(obs, goal)
    assert result.reliable and result.distance_m == 4


def test_pixel_outside_bbox_or_depth_coverage_is_rejected():
    obs = wall_observation(4)
    outside = replace(VISIBLE, bbox=(.6, .6, .75, .75))
    assert not DepthGeometry(0.5, 200, True).estimate(obs, outside).reliable
    obs = wall_observation(4, depth_size=(64, 64))
    obs = replace(obs, depth_intrinsics=intrinsics_from_horizontal_fov(64, 64, 45))
    goal = replace(VISIBLE, bbox=(.7, .2, .95, .8), surface_point=(.9, .5))
    result = DepthGeometry(0.5, 200, True).estimate(obs, goal)
    assert not result.reliable and "outside depth coverage" in result.reason


def test_direct_depth_uses_floored_calibrated_pixel():
    obs = wall_observation(8)
    obs.depth_meters[:] = 8
    obs.depth_meters[67, 65] = 4.25
    result = DepthGeometry(0.5, 200, True).estimate(obs, replace(VISIBLE, surface_point=(0.513, 0.527)))
    assert result.reliable and result.distance_m == pytest.approx(4.25)
    assert "pixel=[65,67]" in result.reason


def test_calibrated_resolution_and_world_rotation_are_preserved():
    obs = wall_observation(6, WorldPose(2, 3, -4, 90), size=(192, 108), depth_size=(256, 144))
    result = DepthGeometry(0.5, 200, True).estimate(obs, replace(VISIBLE, surface_point=(0.6, 0.5)))
    assert result.reliable and result.world_point == pytest.approx((.8, 9, -4), abs=.05)


