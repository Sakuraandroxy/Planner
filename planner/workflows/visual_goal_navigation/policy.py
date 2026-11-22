"""Per-task policy for visual-goal navigation: preflight admission and arrival evaluation."""
import math
from dataclasses import dataclass

from planner.domain import (
    ApproachAssessment,
    ApproachSettings,
    ApproachStatus,
    ApproachTarget,
)
from planner.domain import MissionStage, TaskKind
from planner.domain import WorldPose
from planner.domain import VisualGoalNavigationParameters
from planner.domain import VisualGoalTriggerRules
from planner.domain import VisualGoalLimits
from planner.errors import ProtocolError


@dataclass(frozen=True)
class VisualGoalStageValidator:
    limits: VisualGoalLimits
    trigger_rules: VisualGoalTriggerRules = VisualGoalTriggerRules()

    def validate(self, stage: MissionStage) -> None:
        if (
            stage.kind is not TaskKind.VISUAL_GOAL_NAVIGATION
            or not isinstance(
                stage.parameters, VisualGoalNavigationParameters
            )
        ):
            raise ProtocolError("visual goal stage has invalid parameters")
        try:
            self.limits.validate_arrival_radius(self.trigger_rules.radius_m)
        except ValueError as exc:
            raise ProtocolError(str(exc)) from exc


class TargetApproach:
    @staticmethod
    def distance_within_radius(distance_m: float | None, settings: ApproachSettings) -> bool:
        try:
            valid = (not isinstance(distance_m, bool) and isinstance(distance_m, (int, float))
                     and math.isfinite(distance_m))
        except OverflowError:
            valid = False
        return bool(valid and distance_m >= 0 and distance_m+settings.uncertainty_margin_m <= settings.arrival_radius_m)

    def assess(self, pose: WorldPose, target: ApproachTarget, settings: ApproachSettings) -> ApproachAssessment:
        if not target.reliable or target.position_world is None:
            return ApproachAssessment(ApproachStatus.UNCONFIRMED, None, "target position is unavailable")
        distance = math.dist((pose.x, pose.y, pose.z), target.position_world)
        inside = self.distance_within_radius(distance, settings)
        return ApproachAssessment(ApproachStatus.WITHIN_RADIUS if inside else ApproachStatus.APPROACHING,
            distance, "inside arrival radius; completion is separate" if inside else "outside arrival radius")