import pytest

from planner.domain.execution import StopResult
from planner.domain.visual_goal import (
    ArrivalRelation,
    VisualGoalNavigationParameters,
    VisualGoalNavigationResult,
    VisualGoalStatus,
)


def _parameters(**overrides):
    values = dict(
        instruction="go",
        target_description="the target",
        arrival_relation=ArrivalRelation.NEAR,
        keep_target_in_view=True,
    )
    values.update(overrides)
    return VisualGoalNavigationParameters(**values)


def test_reference_defaults():
    parameters = _parameters()
    assert parameters.arrival_relation is ArrivalRelation.NEAR
    assert parameters.keep_target_in_view is True


@pytest.mark.parametrize(
    "overrides",
    [
        {"instruction": ""},
        {"instruction": "   "},
        {"instruction": 3},
        {"target_description": ""},
        {"target_description": None},
        {"arrival_relation": "near"},
        {"keep_target_in_view": 1},
    ],
)
def test_invalid_parameters_are_rejected(overrides):
    with pytest.raises(ValueError):
        _parameters(**overrides)


def test_statuses_match_the_reference_contract():
    assert {status.value for status in VisualGoalStatus} == {
        "reached",
        "target_not_found",
        "target_lost",
        "target_ambiguous",
        "perception_unreliable",
        "blocked",
        "stuck",
        "timeout",
        "model_error",
        "execution_error",
        "observation_error",
        "cancelled",
    }


def test_result_defaults():
    result = VisualGoalNavigationResult(
        status=VisualGoalStatus.TIMEOUT,
        reason="not confirmed",
        final_distance_m=6.0,
        planning_calls=1,
        elapsed_s=0.5,
        path_length_m=2.0,
        confirmation_count=0,
        stop=StopResult(True),
    )
    assert result.completion_checks == 0
    assert result.completion_confirmed is False
    assert result.evidence_source
    assert result.remaining_motion.segments == ()