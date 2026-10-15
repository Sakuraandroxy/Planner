"""BBox-center protocol and direct execution contracts; no model or simulator I/O."""
from dataclasses import fields, replace
import json
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

from config import load_config
from planner.adapters.target_detection.client import parse_target_detection
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.capabilities.targeting.association import make_reference
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.domain.target_detection import TargetDetection, TargetDetectionInput
from planner.domain.visual_goal import GoalVisibility, VisualGoalPlanningContext
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ConfigurationError, ProtocolError
from tests.fakes.configuration import MAIN_CONFIG, main_config_data, write_config
from tests.fakes.target_detection import detection_json
from tests.fakes.visual_goal import wall_observation


def test_bbox_center_is_the_only_point_and_uses_its_exact_depth():
    detector = Mock()
    detector.detect.return_value = parse_target_detection('[[0.2,0.2],[0.8,0.8]]')
    observation = wall_observation(6)
    observation.depth_meters[:] = 95
    observation.depth_meters[64,64] = 21
    goal = BBoxGoalObserver(detector).observe(observation, "first car")
    assert goal.surface_point == (.5,.5)
    assert not hasattr(goal, "surface_candidates")
    geometry = DepthGeometry(0.5, 200, True).estimate(observation, goal)
    assert geometry.reliable and geometry.distance_m == 21
    assert geometry.world_point == (21,0,-4)


def test_reference_image_matches_identity_but_does_not_lock_a_physical_surface_point():
    detector = Mock()
    detector.detect.return_value = parse_target_detection(detection_json())
    observer = BBoxGoalObserver(detector)
    first_observation = wall_observation(8)
    first_goal = observer.observe(first_observation,"car")
    geometry = DepthGeometry(.5,200,True)
    reference = make_reference(first_observation,first_goal,
        geometry.estimate(first_observation, first_goal))
    detector.detect.return_value = parse_target_detection('[[0.6,0.2],[0.9,0.7]]')
    current = wall_observation(3)
    goal = observer.observe(current,"car",reference)
    # The old anchor projects outside this box; a current bbox center is still used.
    assert goal.status is GoalVisibility.VISIBLE and goal.surface_point == pytest.approx((.75,.45))
    assert goal.association_source == "model_reference_bbox_center"
    assert detector.detect.call_args.args[0].reference_rgb is reference.crop
    assert geometry.estimate(current, goal).reliable


@pytest.mark.parametrize("raw", ['{}','{"bbox":[[0,0],[1,1]],"surface_points":{}}',
    '[[0,0],[1,1],[0.5,0.5]]', '[[true,0],[1,1]]','[[0,0],[NaN,1]]'])
def test_invalid_or_old_five_point_response_never_becomes_a_bbox(raw):
    with pytest.raises(ProtocolError):
        parse_target_detection(raw)


def test_detection_has_no_surface_point_output_fields():
    assert {field.name for field in fields(TargetDetection)} == {"found","bbox"}
    assert not parse_target_detection('[]').found


REMOVED_LIMITS = ("maximum_surface_candidates","maximum_round_distance_m","maximum_round_motion_s",
    "reobserve_heading_change_deg","safety_replan_count","nominal_obstacle_clearance_m","vehicle_radius_m",
    "tracking_margin_m","initial_clearance_radius_m","association_gate_m","safety_sample_spacing_m",
    "execution_overhead_s","search_clearance_confirmed")


@pytest.mark.parametrize("name", REMOVED_LIMITS)
def test_removed_audit_limits_require_explicit_config_migration(tmp_path,name):
    raw = main_config_data()
    raw["visual_goal_navigation"]["limits"][name] = 1
    with pytest.raises(ConfigurationError,match=name):
        load_config(write_config(tmp_path/"obsolete.yaml",raw))


def test_initial_clearance_flag_is_removed(tmp_path):
    raw = main_config_data()
    raw["visual_goal_navigation"]["initial_clearance_confirmed"] = True
    with pytest.raises(ConfigurationError,match="initial_clearance_confirmed"):
        load_config(write_config(tmp_path/"obsolete.yaml",raw))


def test_main_settings_and_context_no_longer_describe_a_route_audit():
    config = load_config(MAIN_CONFIG)
    assert config.target_detection.max_completion_tokens == 64
    assert config.target_detection.reasoning_effort == "none"
    assert not config.visual_goal.collision_check_enabled
    assert not {f.name for f in fields(VisualGoalLimits)}.intersection(REMOVED_LIMITS)
    assert not {f.name for f in fields(VisualGoalPlanningContext)}.intersection(
        {"maximum_distance_m","maximum_motion_s","reobserve_heading_deg","minimum_clearance_m"})
    for path in ("planner/services/trajectory_safety.py","planner/domain/trajectory_safety.py",
                 "planner/domain/target_surface.py","planner/capabilities/targeting/surface_selection.py"):
        assert not Path(path).exists()
