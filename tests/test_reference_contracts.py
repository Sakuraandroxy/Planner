import math

import pytest

from planner.workflows.visual_goal_navigation import ApproachSettings, ApproachStatus, approach_settings
from planner.domain import DepthForegroundSettings
from planner.domain import (
    CompletionBasis,
    ResultMeasurement,
    StageResult,
    StageSummary,
)
from planner.domain import TargetPositionState
from planner.domain import (
    GoalObservation,
    GoalVisibility,
)
from planner.domain import VisualGoalTriggerRules
from planner.domain import VisualGoalLimits
from planner.errors import ExecutionError, ModelCallTimeout, ResourceCleanupError


def _goal() -> GoalObservation:
    return GoalObservation(
        status=GoalVisibility.VISIBLE,
        bbox=(0.4, 0.4, 0.6, 0.7),
        surface_point=(0.5, 0.6),
        description="the red car",
        evidence="visible in the centre",
    )


def _target_state(**overrides) -> TargetPositionState:
    values = dict(
        stage_id="stage-1",
        anchor_world_point=(1.0, 2.0, 0.0),
        world_point=(4.0, 6.0, 0.0),
        capture_id="capture-1",
        captured_monotonic=1.0,
        timestamp_ns=10,
    )
    values.update(overrides)
    return TargetPositionState(**values)


def test_goal_observation_defaults_follow_the_reference_contract():
    goal = _goal()
    assert goal.same_target is True
    assert goal.association_source == "local_geometry"
    assert goal.surface_points is None


def test_approach_settings_reject_margin_at_or_above_radius():
    with pytest.raises(ValueError):
        ApproachSettings(5.0, 5.0)
    assert ApproachStatus.WITHIN_RADIUS.value == "within_radius"


def test_depth_foreground_settings_validate_radius_and_ratio():
    DepthForegroundSettings()
    with pytest.raises(ValueError):
        DepthForegroundSettings(pixel_radius=0)
    with pytest.raises(ValueError):
        DepthForegroundSettings(depth_gap_ratio=1.0)


def test_trigger_rules_default_pause_to_the_single_radius():
    rules = VisualGoalTriggerRules()
    assert rules.effective_pause_radius_m == rules.radius_m
    assert len(rules.required_conditions) == 3
    tighter = VisualGoalTriggerRules(radius_m=7.0, pause_radius_m=6.5)
    assert tighter.effective_pause_radius_m == 6.5
    with pytest.raises(ValueError):
        VisualGoalTriggerRules(maximum_completion_checks=0)


def test_limits_derive_approach_settings_and_validate_search():
    limits = VisualGoalLimits(distance_uncertainty_margin_m=0.2)
    approach = approach_settings(limits, 7.0)
    assert approach.arrival_radius_m == 7.0
    assert approach.uncertainty_margin_m == 0.2
    assert math.isclose(limits.target_position_update_radius_m, 0.5)
    with pytest.raises(ValueError):
        VisualGoalLimits(confirmation_count=1)
    with pytest.raises(ValueError):
        VisualGoalLimits(search_half_span_deg=100.0)
    with pytest.raises(ValueError):
        VisualGoalLimits(search_step_deg=5.0, search_half_span_deg=90.0)


def test_target_state_requires_matching_surface_sets_and_known_surfaces():
    state = _target_state()
    assert state.known_surfaces == ((4.0, 6.0, 0.0),)
    bound = _target_state(
        anchor_surface_points_world=((1.0, 2.0, 0.0),),
        surface_points_world=((4.0, 6.0, 0.0),),
    )
    assert bound.known_surfaces == ((4.0, 6.0, 0.0),)
    with pytest.raises(ValueError):
        _target_state(
            anchor_surface_points_world=((1.0, 2.0, 0.0),),
            surface_points_world=(),
        )
    with pytest.raises(ValueError):
        _target_state(captured_monotonic=0.0)


def test_stage_summary_requires_evidence_for_confirmed_completion():
    summary = StageSummary(
        status="reached",
        reason="inside radius",
        completion_basis=CompletionBasis.MODEL_WITH_LOCAL_CHECKS,
        completion_confirmed=True,
        measurements=(ResultMeasurement("distance_m", 6.0, "m"),),
    )
    result = StageResult(stage_id="stage-1", success=True, summary=summary)
    assert result.success is True
    with pytest.raises(ValueError):
        StageSummary(status="reached", reason="x", completion_confirmed=True)
    with pytest.raises(ValueError):
        StageResult(
            stage_id="stage-1",
            success=False,
            summary=summary,
        )


def test_model_timeout_is_an_execution_error_and_cleanup_reports_failures():
    assert issubclass(ModelCallTimeout, ExecutionError)
    error = ResourceCleanupError((RuntimeError("camera"), ValueError("actor")))
    assert len(error.failures) == 2
    assert "RuntimeError" in str(error)
    assert "ValueError" in str(error)