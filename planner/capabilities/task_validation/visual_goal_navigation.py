"""Preflight validation for static visual-goal navigation."""
from dataclasses import dataclass

from planner.domain.mission import MissionStage, TaskKind
from planner.domain.visual_goal import VisualGoalNavigationParameters
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.visual_goal_completion import VisualGoalTriggerRules
from planner.errors import ProtocolError


@dataclass(frozen=True)
class VisualGoalStageValidator:
    limits: VisualGoalLimits
    maximum_depth_m: float = 200.0
    trigger_rules: VisualGoalTriggerRules = VisualGoalTriggerRules()

    def validate(self, stage: MissionStage) -> None:
        if stage.kind is not TaskKind.VISUAL_GOAL_NAVIGATION or not isinstance(stage.parameters, VisualGoalNavigationParameters):
            raise ProtocolError("visual goal stage has invalid parameters")
        try:
            self.limits.validate_arrival_radius(self.trigger_rules.radius_m, self.maximum_depth_m)
        except ValueError as exc:
            raise ProtocolError(str(exc)) from exc
