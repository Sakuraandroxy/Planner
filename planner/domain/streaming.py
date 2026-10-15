"""Measured streaming state and immutable pause records; no device ownership."""
from dataclasses import dataclass
import math

from .pose import WorldPose
from .sliding_window import QueuedWaypoint
from .execution import ExecutionRecord


@dataclass(frozen=True)
class TimedVehicleState:
    pose: WorldPose
    timestamp_ns: int
    observed_monotonic: float
    velocity_world: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self):
        if type(self.timestamp_ns) is not int or self.timestamp_ns <= 0:
            raise ValueError("stream state requires a positive simulator timestamp")
        if isinstance(self.observed_monotonic, bool) or not math.isfinite(self.observed_monotonic) or len(self.velocity_world) != 3 or not all(
            not isinstance(v, bool) and math.isfinite(v) for v in self.velocity_world
        ):
            raise ValueError("stream state requires finite measurements")


@dataclass(frozen=True)
class QueuePauseSnapshot:
    stage_token: int
    queue_revision: int
    state: TimedVehicleState
    unwrapped_yaw_deg: float
    remaining: tuple[QueuedWaypoint, ...]
    first_goal_yaw_deg: float | None
    reason: str


@dataclass(frozen=True)
class QueueObservationAnchor:
    stage_token: int
    queue_revision: int
    timestamp_ns: int
    items: tuple[QueuedWaypoint, ...]
    history: ExecutionRecord | None = None

