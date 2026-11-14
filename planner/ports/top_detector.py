"""Typed top detector; the normal near-target detector contract stays unchanged."""
from typing import Protocol
from planner.domain.visual_goal_above import AboveDetection, AboveDetectionInput


class TopDetector(Protocol):
    def detect(self, request: AboveDetectionInput, *, timeout_s: float) -> AboveDetection: ...
