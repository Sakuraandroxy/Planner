"""Immutable continuation anchors; no network or vehicle ownership here."""
from dataclasses import dataclass
import math

from .observation import Observation
from .pose import WorldPose
from .trajectory import RelativeTrajectory, WorldTrajectory


@dataclass(frozen=True)
class SlidingWindowSettings:
    enabled: bool = False
    refill_pending_count: int = 3
    latency_margin_s: float = 1.5
    max_task_duration_s: float = 600.0
    max_consecutive_plan_failures: int = 2
    max_observation_age_s: float = 120.0

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise ValueError("sliding_window.enabled must be a boolean")
        if type(self.refill_pending_count) is not int or not 1 <= self.refill_pending_count <= 5:
            raise ValueError("refill_pending_count must be an integer between 1 and 5")
        if type(self.max_consecutive_plan_failures) is not int or not 1 <= self.max_consecutive_plan_failures <= 5:
            raise ValueError("max_consecutive_plan_failures must be an integer between 1 and 5")
        for name in ("latency_margin_s", "max_task_duration_s", "max_observation_age_s"):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class QueuedWaypoint:
    waypoint_id: int
    start: WorldPose
    end: WorldPose
    yaw_delta_deg: float


@dataclass(frozen=True)
class PlanningSnapshot:
    stage_token: int
    request_id: int
    observation: Observation
    initial_pose: WorldPose
    instruction: str
    guidance: str | None
    guidance_start: WorldPose | None
    pending: RelativeTrajectory
    pending_ids: tuple[int, ...]
    queue_revision: int
    tail: WorldPose
    history: RelativeTrajectory | None
    history_start: WorldPose | None
    created_at: float
    request_deadline: float
    valid_until: float


@dataclass(frozen=True)
class ContinuationCandidate:
    snapshot: PlanningSnapshot
    world: WorldTrajectory


@dataclass(frozen=True)
class PlanningOutput:
    relative: RelativeTrajectory
    latency_s: float
    completed_at: float
