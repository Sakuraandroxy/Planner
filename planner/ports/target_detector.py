"""Target detector port."""

from __future__ import annotations

from typing import Protocol

from planner.domain import TargetDetection, TargetDetectionInput


class TargetDetector(Protocol):
    def detect(
        self, request: TargetDetectionInput, *, timeout_s: float
    ) -> TargetDetection: ...