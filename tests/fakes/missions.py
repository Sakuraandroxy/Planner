"""Test assembly helper; production MissionValidator contains no task branches."""
from planner.application.mission_validator import MissionValidator as CommonMissionValidator
from planner.capabilities.task_validation.basic_motion import BasicMotionStageValidator
from planner.capabilities.task_validation.visual_goal_navigation import VisualGoalStageValidator
from planner.domain.mission import TaskKind
from planner.domain.visual_goal_completion import VisualGoalTriggerRules


def mission_validator(supported_kinds, visual_goal_limits=None, maximum_depth_m=200.0, *,
                      visual_goal_trigger_rules=None):
    validators = {}
    if TaskKind.BASIC_MOTION in supported_kinds:
        validators[TaskKind.BASIC_MOTION] = BasicMotionStageValidator()
    if TaskKind.VISUAL_GOAL_NAVIGATION in supported_kinds and visual_goal_limits is not None:
        validators[TaskKind.VISUAL_GOAL_NAVIGATION] = VisualGoalStageValidator(
            visual_goal_limits, maximum_depth_m, visual_goal_trigger_rules or VisualGoalTriggerRules())
    return CommonMissionValidator(validators)
