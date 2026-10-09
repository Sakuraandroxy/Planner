"""Immutable contracts for static visual-goal approach; no device/model calls."""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Any

from .pose import WorldPose
from .execution import StopResult


class ArrivalRelation(str, Enum):
    NEAR = "near"


@dataclass(frozen=True)
class VisualGoalNavigationParameters:
    instruction: str
    target_description: str
    arrival_relation: ArrivalRelation = ArrivalRelation.NEAR
    standoff_m: float | None = None
    keep_target_in_view: bool = True

    def __post_init__(self):
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("visual goal instruction must be non-empty text")
        if not isinstance(self.target_description, str) or not self.target_description.strip():
            raise ValueError("target description must be non-empty text")
        if self.arrival_relation is not ArrivalRelation.NEAR:
            raise ValueError("only near arrival is supported")
        if self.standoff_m is not None and (
            isinstance(self.standoff_m, bool) or not math.isfinite(self.standoff_m) or self.standoff_m <= 0
        ):
            raise ValueError("standoff must be finite and positive")
        if not isinstance(self.keep_target_in_view, bool):
            raise ValueError("keep_target_in_view must be boolean")


class GoalVisibility(str, Enum):
    VISIBLE = "visible"
    NOT_VISIBLE = "not_visible"
    AMBIGUOUS = "ambiguous"


@dataclass(frozen=True)
class GoalObservation:
    """One locally selected region; not the raw goal_observation_v2 response."""
    status: GoalVisibility
    bbox: tuple[float, float, float, float] | None
    surface_point: tuple[float, float] | None
    description: str
    evidence: str
    same_target: bool = True


@dataclass(frozen=True)
class GoalGeometry:
    reliable: bool
    reason: str
    distance_m: float | None = None
    world_point: tuple[float, float, float] | None = None
    valid_ratio: float = 0.0
    mad_m: float | None = None


@dataclass(frozen=True)
class GoalReference:
    description: str
    crop: Any
    world_point: tuple[float, float, float]
    bbox: tuple[float, float, float, float]
    region_crop: Any
    region_description: str
    region_point: tuple[float, float]  # Normalized within region_crop, not the current image.


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
    standoff_m: float
    tolerance_m: float
    maximum_distance_m: float
    maximum_motion_s: float
    maximum_vertical_offset_m: float
    reobserve_heading_deg: float
    minimum_clearance_m: float


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
