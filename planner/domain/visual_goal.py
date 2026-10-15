"""Immutable contracts for static visual-goal approach; no device/model calls."""
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .pose import WorldPose
from .execution import StopResult
from .trajectory import MotionTrajectory


class ArrivalRelation(str, Enum):
    NEAR = "near"


@dataclass(frozen=True)
class VisualGoalNavigationParameters:
    instruction: str
    target_description: str
    arrival_relation: ArrivalRelation = ArrivalRelation.NEAR
    keep_target_in_view: bool = True

    def __post_init__(self):
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("visual goal instruction must be non-empty text")
        if not isinstance(self.target_description, str) or not self.target_description.strip():
            raise ValueError("target description must be non-empty text")
        if self.arrival_relation is not ArrivalRelation.NEAR:
            raise ValueError("only near arrival is supported")
        if not isinstance(self.keep_target_in_view, bool):
            raise ValueError("keep_target_in_view must be boolean")


class GoalVisibility(str, Enum):
    VISIBLE = "visible"
    NOT_VISIBLE = "not_visible"
    AMBIGUOUS = "ambiguous"


@dataclass(frozen=True)
class GoalObservation:
    """One current bbox-center point; target identity is provided by the observer."""
    status: GoalVisibility
    bbox: tuple[float, float, float, float] | None
    surface_point: tuple[float, float] | None
    description: str
    evidence: str
    same_target: bool = True
    association_source: str = "local_geometry"


@dataclass(frozen=True)
class GoalGeometry:
    """Validity of one calibrated depth estimate, without scene/neighborhood audit."""
    reliable: bool
    reason: str
    distance_m: float | None = None
    world_point: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class GoalReference:
    """Initial appearance reference plus the latest observed bbox-center position."""
    description: str
    crop: Any
    world_point: tuple[float, float, float]
    bbox: tuple[float, float, float, float]


class VisualGoalState(str, Enum):
    SEARCHING = "searching"
    APPROACHING = "approaching"
    VERIFYING = "verifying"


class VisualGoalStatus(str, Enum):
    REACHED = "reached"
    TARGET_NOT_FOUND = "target_not_found"
    TARGET_LOST = "target_lost"
    TARGET_AMBIGUOUS = "target_ambiguous"
    TARGET_MOVED = "target_moved"
    PERCEPTION_UNRELIABLE = "perception_unreliable"
    BLOCKED = "blocked"
    STUCK = "stuck"
    TIMEOUT = "timeout"
    SAFETY_STOP = "safety_stop"
    MODEL_ERROR = "model_error"
    EXECUTION_ERROR = "execution_error"
    OBSERVATION_ERROR = "observation_error"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class VisualGoalPlanningContext:
    target_description: str
    goal: GoalObservation
    geometry: GoalGeometry
    state: VisualGoalState
    arrival_radius_m: float
    distance_uncertainty_margin_m: float
    maximum_vertical_offset_m: float | None
    goal_observation_is_current: bool = True
    target_capture_id: str = ""
    target_captured_monotonic: float | None = None
    keep_target_in_view: bool = True


@dataclass(frozen=True)
class VisualGoalNavigationResult:
    status: VisualGoalStatus
    reason: str
    final_distance_m: float | None
    planning_calls: int
    elapsed_s: float
    path_length_m: float
    confirmation_count: int
    stop: StopResult
    evidence_source: str = "VLM association + depth estimate + stopped observations"
    run_directory: str | None = None
    path_length_source: str = "sum_of_measured_segment_chords_not_continuous_path"
    completion_checks: int = 0
    completion_confirmed: bool = False
    remaining_motion: MotionTrajectory = MotionTrajectory(())
