import math

import pytest

from planner.domain.approach import ApproachSettings, ApproachStatus
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.depth_foreground import (
    DepthForegroundResult,
    DepthForegroundSettings,
)
from planner.domain.free_navigation import FreeNavigationParameters
from planner.domain.mission import (
    MissionPlan,
    MissionStage,
    TaskKind,
)
from planner.domain.result import (
    CompletionBasis,
    MissionResult,
    ResultMeasurement,
    StageResult,
    StageSummary,
)
from planner.domain.target_state import TargetPositionState
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalReference,
    GoalVisibility,
    VisualGoalNavigationParameters,
    VisualGoalPlanningContext,
    VisualGoalState,
)
from planner.domain.visual_goal_completion import VisualGoalTriggerRules
from planner.domain.visual_goal_limits import VisualGoalLimits
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


def test_goal_geometry_carries_depth_filter_and_bound_surfaces():
    geometry = GoalGeometry(
        reliable=True,
        reason="bound",
        distance_m=5.0,
        world_point=(1.0, 2.0, 0.0),
        depth_filter=DepthForegroundResult(applied=False, reason="disabled"),
        surface_points_world=((1.0, 2.0, 0.0),),
    )
    assert geometry.depth_filter.applied is False
    assert geometry.surface_points_world == ((1.0, 2.0, 0.0),)


def test_goal_reference_keeps_the_initial_crop_not_the_latest_estimate():
    crop = object()
    reference = GoalReference(
        description="the red car",
        crop=crop,
        world_point=(3.0, 4.0, 0.0),
        bbox=(0.4, 0.4, 0.6, 0.7),
    )
    assert reference.crop is crop


def test_planning_context_exposes_state_and_radius():
    context = VisualGoalPlanningContext(
        target_description="the red car",
        goal=_goal(),
        geometry=GoalGeometry(reliable=True, reason="fresh"),
        state=VisualGoalState.APPROACHING,
        arrival_radius_m=7.0,
        distance_uncertainty_margin_m=0.0,
        maximum_vertical_offset_m=None,
    )
    assert context.state is VisualGoalState.APPROACHING
    assert context.goal_observation_is_current is True


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
    approach = limits.approach_settings(7.0)
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


def test_mission_types_carry_visual_goal_stages():
    parameters = VisualGoalNavigationParameters(
        instruction="drive to the red car",
        target_description="the red car",
    )
    stage = MissionStage(
        stage_id="stage-1",
        kind=TaskKind.VISUAL_GOAL_NAVIGATION,
        parameters=parameters,
    )
    plan = MissionPlan(protocol_version="1", stages=(stage,))
    assert plan.stages[0].kind is TaskKind.VISUAL_GOAL_NAVIGATION
    assert MissionResult(success=True, stages=()).stages == ()


def test_mission_types_carry_basic_motion_stages():
    parameters = BasicMotionParameters(
        instruction="drive forward 5 metres",
        forward_m=5.0,
    )
    stage = MissionStage(
        stage_id="stage-1",
        kind=TaskKind.BASIC_MOTION,
        parameters=parameters,
    )
    plan = MissionPlan(protocol_version="mission_plan_v1", stages=(stage,))
    assert plan.stages[0].kind is TaskKind.BASIC_MOTION
    assert plan.stages[0].parameters.forward_m == 5.0


def test_mission_types_carry_free_navigation_stages():
    parameters = FreeNavigationParameters(
        instruction="keep driving along the road and avoid obstacles",
    )
    stage = MissionStage(
        stage_id="stage-3",
        kind=TaskKind.FREE_NAVIGATION,
        parameters=parameters,
    )
    plan = MissionPlan(protocol_version="mission_plan_v1", stages=(stage,))
    assert plan.stages[0].kind is TaskKind.FREE_NAVIGATION
    assert plan.stages[0].parameters.instruction.startswith("keep driving")


def test_model_timeout_is_an_execution_error_and_cleanup_reports_failures():
    assert issubclass(ModelCallTimeout, ExecutionError)
    error = ResourceCleanupError((RuntimeError("camera"), ValueError("actor")))
    assert len(error.failures) == 2
    assert "RuntimeError" in str(error)
    assert "ValueError" in str(error)