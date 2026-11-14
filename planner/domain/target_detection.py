"""Model-facing target detection types: normalized boxes and surface pixels.

Nothing in this module knows about world coordinates. The detector only binds
image evidence; depth back-projection happens later in
``capabilities.targeting.depth_geometry``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .observation import Observation


@dataclass(frozen=True)
class NormalizedBoundingBox:
    """Box corners normalized to the full image, origin top-left."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def __post_init__(self) -> None:
        values = (self.xmin, self.ymin, self.xmax, self.ymax)
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in values
        ):
            raise ValueError("bbox requires finite numeric coordinates")
        if not (
            0.0 <= self.xmin < self.xmax <= 1.0
            and 0.0 <= self.ymin < self.ymax <= 1.0
        ):
            raise ValueError("bbox requires ordered nonempty normalized corners")

    @property
    def coordinates(self) -> tuple[float, float, float, float]:
        return (self.xmin, self.ymin, self.xmax, self.ymax)


@dataclass(frozen=True)
class TargetDetection:
    found: bool
    bbox: NormalizedBoundingBox | None = None
    surface_points: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        if type(self.found) is not bool:
            raise ValueError("found must be boolean")
        if self.found and not isinstance(self.bbox, NormalizedBoundingBox):
            raise ValueError("a found target requires a normalized bbox")
        if not self.found and self.bbox is not None:
            raise ValueError("a missing target cannot carry a bbox")
        if not isinstance(self.surface_points, tuple) or len(self.surface_points) > 5:
            raise ValueError("surface_points must be a tuple of at most 5 points")
        if not self.found and self.surface_points:
            raise ValueError("a missing target cannot carry surface points")
        for point in self.surface_points:
            if (
                not isinstance(point, tuple)
                or len(point) != 2
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    for value in point
                )
            ):
                raise ValueError("surface points require finite coordinate pairs")
            if not (
                self.bbox.xmin <= point[0] <= self.bbox.xmax
                and self.bbox.ymin <= point[1] <= self.bbox.ymax
            ):
                raise ValueError("surface point is outside the target bbox")

    @classmethod
    def missing(cls) -> "TargetDetection":
        return cls(found=False)

    @property
    def usable_points(self) -> tuple[tuple[float, float], ...]:
        """Exactly the model-selected points; no generated bbox-centre fallback.

        An empty array means the detector identified the target but could not
        confidently bind a solid surface pixel. Sampling the bbox centre would
        risk background or holes, so geometry treats it as unusable.
        """
        return self.surface_points


@dataclass(frozen=True)
class TargetDetectionInput:
    current: Observation
    instruction: str
    target_description: str
    reference_rgb: Any = None