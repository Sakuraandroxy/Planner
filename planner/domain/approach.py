"""Reusable arrival-radius contracts; no trajectory authorization or body envelope."""
from dataclasses import dataclass
from enum import Enum
import math

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
