"""Per-task policy for visual-goal navigation: preflight admission and arrival evaluation."""
import math
from dataclasses import dataclass
from enum import Enum

from planner.domain import MissionStage, TaskKind
from planner.domain import WorldPose
from planner.domain import VisualGoalNavigationParameters
from planner.domain import VisualGoalTriggerRules
from planner.domain import VisualGoalLimits
from planner.errors import ProtocolError

WorldPoint = tuple[float, float, float]


def _finite(value) -> bool:
    try:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class ApproachSettings:
    arrival_radius_m: float
    uncertainty_margin_m: float

    def __post_init__(self):
        if not (_finite(self.arrival_radius_m) and self.arrival_radius_m > 0 and
                _finite(self.uncertainty_margin_m) and 0 <= self.uncertainty_margin_m < self.arrival_radius_m):
            raise ValueError("arrival radius must be positive and exceed finite nonnegative uncertainty")


@dataclass(frozen=True)
class ApproachTarget:
    position_world: WorldPoint | None
    reliable: bool

    def __post_init__(self):
        if type(self.reliable) is not bool or (self.reliable and self.position_world is None):
            raise ValueError("reliable approach target requires a world position")
        point = self.position_world
        if point is not None and (not isinstance(point, tuple) or len(point) != 3 or not all(_finite(v) for v in point)):
            raise ValueError("target must be a finite immutable world-coordinate triple")


class ApproachStatus(str, Enum):
    UNCONFIRMED = "unconfirmed"
    APPROACHING = "approaching"
    WITHIN_RADIUS = "within_radius"


@dataclass(frozen=True)
class ApproachAssessment:
    status: ApproachStatus
    distance_m: float | None
    reason: str


def approach_settings(limits: VisualGoalLimits, radius_m: float) -> ApproachSettings:
    """Arrival radius with the stage's uncertainty margin; rejects both out of range."""
    return ApproachSettings(radius_m, limits.distance_uncertainty_margin_m)


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
            approach_settings(self.limits, self.trigger_rules.radius_m)
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