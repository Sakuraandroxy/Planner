"""Measured streaming state and the immutable image queue anchor; no device ownership."""

import math
from dataclasses import dataclass

from .execution import ExecutionRecord
from .pose import WorldPose
from .sliding_window import QueuedWaypoint


@dataclass(frozen=True)
class TimedVehicleState:
    pose: WorldPose
    timestamp_ns: int
    observed_monotonic: float
    velocity_world: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pitch_deg: float | None = None
    roll_deg: float | None = None
    #: 实测偏航角速度（度/秒）。航向伺服用它做阻尼，不再用位姿差分。
    #: 引擎不提供时保持 None，伺服会退回无阻尼。
    yaw_rate_deg_s: float | None = None

    def __post_init__(self) -> None:
        if type(self.timestamp_ns) is not int or self.timestamp_ns <= 0:
            raise ValueError(
                "stream state requires a positive simulator timestamp"
            )
        if (
            isinstance(self.observed_monotonic, bool)
            or not math.isfinite(self.observed_monotonic)
            or len(self.velocity_world) != 3
            or not all(
                not isinstance(v, bool) and math.isfinite(v)
                for v in self.velocity_world
            )
        ):
            raise ValueError("stream state requires finite measurements")
        for value in (self.pitch_deg, self.roll_deg, self.yaw_rate_deg_s):
            if value is not None and (
                isinstance(value, bool) or not math.isfinite(value)
            ):
                raise ValueError("stream attitude must be finite when supplied")


@dataclass(frozen=True)
class QueueObservationAnchor:
    stage_token: int
    queue_revision: int
    timestamp_ns: int
    items: tuple[QueuedWaypoint, ...]
    history: ExecutionRecord | None = None
