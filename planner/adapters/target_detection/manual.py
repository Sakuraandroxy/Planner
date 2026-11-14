"""Manual detector: trust a human-supplied box/point for live M3-M4 checks.

This adapter exists so the depth back-projection and trajectory loop can be
exercised against a real CARLA scene without a vision endpoint. It is not a
stand-in for visual grounding in the final system.
"""

from __future__ import annotations

from ...domain.target_detection import (
    NormalizedBoundingBox,
    TargetDetection,
    TargetDetectionInput,
)


class ManualTargetDetector:
    def __init__(
        self,
        bbox: NormalizedBoundingBox,
        surface_points: tuple[tuple[float, float], ...] = (),
    ):
        if not surface_points:
            # A human explicitly selected this box; aim at its centre unless
            # the caller supplied surface points.
            surface_points = (
                ((bbox.xmin + bbox.xmax) / 2.0, (bbox.ymin + bbox.ymax) / 2.0),
            )
        self._detection = TargetDetection(
            found=True, bbox=bbox, surface_points=tuple(surface_points)
        )

    @classmethod
    def from_corners(
        cls,
        xmin: float,
        ymin: float,
        xmax: float,
        ymax: float,
        surface_points: tuple[tuple[float, float], ...] = (),
    ) -> "ManualTargetDetector":
        return cls(
            NormalizedBoundingBox(xmin, ymin, xmax, ymax),
            surface_points=surface_points,
        )

    def detect(
        self, request: TargetDetectionInput, *, timeout_s: float
    ) -> TargetDetection:
        return self._detection