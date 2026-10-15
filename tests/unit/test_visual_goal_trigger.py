import pytest

from planner.capabilities.completion.visual_goal_trigger import VisualGoalRadiusTrigger
from planner.domain.visual_goal_completion import VisualGoalTargetSnapshot, VisualGoalTriggerRules
from planner.domain.pose import WorldPose
from tests.fakes.visual_goal import Clock


def test_radius_latch_rearms_on_measured_progress_or_leaving_radius():
    clock = Clock()
    trigger = VisualGoalRadiusTrigger(VisualGoalTriggerRules(radius_m=5), 45, clock)
    trigger.update(VisualGoalTargetSnapshot("stage", (6, 0, 0), clock()))
    assert not trigger.should_pause(WorldPose(0, 0, 0, 0))
    assert trigger.should_pause(WorldPose(1, 0, 0, 0))
    assert not trigger.should_pause(WorldPose(1, 0, 0, 0))
    assert trigger.should_pause(WorldPose(1.5, 0, 0, 0))
    assert not trigger.should_pause(WorldPose(0, 0, 0, 0))
    assert trigger.should_pause(WorldPose(1.5, 0, 0, 0))


def test_expired_reference_forces_observation_and_stage_cannot_change():
    clock = Clock()
    trigger = VisualGoalRadiusTrigger(VisualGoalTriggerRules(), 1, clock)
    trigger.update(VisualGoalTargetSnapshot("stage", (20, 0, 0), clock()))
    clock.sleep(2)
    assert trigger.should_pause(WorldPose(0, 0, 0, 0))
    assert trigger.last_reason == "target_reference_expired"
    with pytest.raises(ValueError, match="another mission stage"):
        trigger.update(VisualGoalTargetSnapshot("other", (20, 0, 0), clock()))


@pytest.mark.parametrize("options", [{"radius_m": True}, {"radius_m": float("nan")},
    {"rearm_translation_m": 0}, {"maximum_completion_checks": 1.5}, {"required_conditions": ()}])
def test_invalid_rules_rejected(options):
    with pytest.raises(ValueError):
        VisualGoalTriggerRules(**options)
