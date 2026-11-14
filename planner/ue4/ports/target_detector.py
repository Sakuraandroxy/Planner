from typing import Protocol
from planner.ue4.domain.target_detection import TargetDetection, TargetDetectionInput


class TargetDetector(Protocol):
    def detect(self, request: TargetDetectionInput, *, timeout_s: float) -> TargetDetection: ...
