"""Radius, parser and completion admission contracts; no model or simulator calls."""
from dataclasses import fields, replace

import pytest

from config import load_config
from planner.adapters.task_parser.prompt import task_parser_system_prompt
from planner.adapters.task_parser.tasks.visual_goal_navigation import decode_visual_goal
from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
from planner.capabilities.completion.visual_goal_request import require_visual_completion_evidence
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.domain.mission import TaskKind
from planner.domain.visual_goal import VisualGoalNavigationParameters
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ConfigurationError, ProtocolError
from tests.fakes.configuration import main_config_data, write_config
from tests.unit.test_visual_goal_completion_client import request


def test_visual_parser_contract_has_no_stopping_distance_and_rules_are_task_owned():
    prompt = task_parser_system_prompt(set(TaskKind))
    assert "standoff_m" not in prompt and "radius_m" not in prompt
    assert "first car to the front-left" in prompt
    assert "object ordinals are not motion quantities" in prompt.lower()
    assert "up/down" in prompt and "Never invent a kind" in prompt
    parsed = decode_visual_goal({"instruction":"Approach the first car to the front-left. 飞到左前方第一辆车旁。",
        "target_description":"the first car to the front-left","arrival_relation":"near","keep_target_in_view":True})
    assert parsed.target_description == "the first car to the front-left"
    assert "standoff_m" not in {f.name for f in fields(VisualGoalNavigationParameters)}


@pytest.mark.parametrize("value", [None,4,True])
def test_legacy_distance_in_visual_parser_is_explicitly_rejected(value):
    with pytest.raises(ProtocolError, match="unknown"):
        decode_visual_goal({"instruction":"approach", "target_description":"car", "standoff_m":value})


@pytest.mark.parametrize("distance,accepted",[(2.3,True),(3.0,True),(3.5,True),(3.8,False),(4.1,False)])
def test_completion_requires_safe_interior_with_error_margin(distance,accepted):
    original=request()
    evidence=replace(original, geometry=replace(original.geometry, distance_m=distance))
    if accepted:
        require_visual_completion_evidence(evidence)
        text=VisualGoalCompletionPrompt.build_user_prompt(evidence)
        assert "圈内位置均可" in text and "±" not in text
    else:
        with pytest.raises(ProtocolError):
            require_visual_completion_evidence(evidence)




@pytest.mark.parametrize("height",[0,-1,True,float("nan"),float("inf")])
def test_invalid_optional_height_boundary_is_rejected(height):
    with pytest.raises(ValueError):
        VisualGoalLimits(maximum_vertical_offset_m=height)
    with pytest.raises(ProtocolError):
        require_visual_completion_evidence(replace(request(),height_tolerance_m=height))


def test_height_boundary_can_be_omitted_or_enabled_without_fixed_height_requirement():
    original=request()
    moved=replace(original.observation,vehicle_pose=replace(original.observation.vehicle_pose,z=-6))
    # Distances here are already supplied measurements; verify height admission independently.
    require_visual_completion_evidence(replace(original,observation=moved,height_tolerance_m=None))
    with pytest.raises(ProtocolError):
        require_visual_completion_evidence(replace(original,observation=moved,height_tolerance_m=1))


@pytest.mark.parametrize("legacy",["standoff_m","distance_tolerance_m","minimum_standoff_m"])
def test_legacy_config_does_not_silently_reactivate_distance_band(tmp_path,legacy):
    raw=main_config_data()
    raw["visual_goal_navigation"]["limits"][legacy]=4
    with pytest.raises(ConfigurationError):
        load_config(write_config(tmp_path/"legacy.yaml",raw))


@pytest.mark.parametrize("key,value", [("progress_window_rounds",4),("minimum_progress_m",.5)])
def test_removed_distance_progress_settings_are_not_silently_accepted(tmp_path,key,value):
    raw=main_config_data()
    raw["visual_goal_navigation"]["limits"][key]=value
    with pytest.raises(ConfigurationError,match=key):
        load_config(write_config(tmp_path/"legacy_progress.yaml",raw))


def test_workflow_configuration_separates_pause_and_completion_radii():
    config=load_config("config/base.yaml")
    assert config.visual_goal.completion_rules.radius_m==7
    assert config.visual_goal.completion_rules.effective_pause_radius_m==6.5
    limits=config.visual_goal.limits
    assert limits.maximum_vertical_offset_m is None
    settings=limits.approach_settings(config.visual_goal.completion_rules.radius_m)
    assert settings.arrival_radius_m==7
    assert settings.uncertainty_margin_m==0
    with pytest.raises(ValueError):
        replace(limits,distance_uncertainty_margin_m=.3).validate_arrival_radius(.3,200)


@pytest.mark.parametrize("distance,accepted", [(6.8,True),(7.0,True),(7.001,False)])
def test_completion_accepts_stopped_distance_outside_pause_radius_up_to_seven(distance,accepted):
    original=request()
    evidence=replace(original, rules=replace(original.rules,radius_m=7,pause_radius_m=6.5),
        uncertainty_margin_m=0,geometry=replace(original.geometry,distance_m=distance))
    if accepted:
        require_visual_completion_evidence(evidence)
        prompt=VisualGoalCompletionPrompt.build_user_prompt(evidence)
        assert "急停触发半径：6.5米" in prompt
        assert "完成距离上限：7米" in prompt
    else:
        with pytest.raises(ProtocolError):
            require_visual_completion_evidence(evidence)


@pytest.mark.parametrize("distance,accepted", [(3.8,True),(4.0,True),(4.001,False)])
def test_zero_margin_uses_same_distance_boundary_for_candidate_and_completion_request(distance,accepted):
    limits=VisualGoalLimits(distance_uncertainty_margin_m=0)
    original=request()
    evidence=replace(original,uncertainty_margin_m=0,
        geometry=replace(original.geometry,distance_m=distance))
    candidate=VisualGoalCompletion(limits).candidate(evidence.parameters,evidence.observation,
        evidence.goal,evidence.geometry,evidence.initial_pose,radius_m=4)
    assert candidate is accepted
    if accepted:
        require_visual_completion_evidence(evidence)
    else:
        with pytest.raises(ProtocolError):
            require_visual_completion_evidence(evidence)


@pytest.mark.parametrize("margin", [-.1,True,float("nan"),float("inf")])
def test_distance_margin_still_rejects_negative_or_non_numeric_values(margin):
    with pytest.raises(ValueError):
        VisualGoalLimits(distance_uncertainty_margin_m=margin)
