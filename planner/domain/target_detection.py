"""Normalized RGB boxes, never model-generated world positions."""
from dataclasses import dataclass
import math
from typing import Any
from .observation import Observation


@dataclass(frozen=True)
class NormalizedBoundingBox:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def __post_init__(self):
        values = (self.xmin, self.ymin, self.xmax, self.ymax)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError("bbox requires finite numeric coordinates")
        if not (0 <= self.xmin < self.xmax <= 1 and 0 <= self.ymin < self.ymax <= 1):
            raise ValueError("bbox requires ordered nonempty normalized corners")

    @property
    def coordinates(self):
        return (self.xmin, self.ymin, self.xmax, self.ymax)


@dataclass(frozen=True)
class TargetDetection:
    found: bool
    bbox: NormalizedBoundingBox | None = None

    def __post_init__(self):
        if type(self.found) is not bool or (
            self.found and not isinstance(self.bbox, NormalizedBoundingBox)
        ) or (not self.found and self.bbox is not None):
            raise ValueError("inconsistent target detection")


@dataclass(frozen=True)
class TargetDetectionInput:
    current: Observation
    instruction: str
    target_description: str
    reference_rgb: Any = None
