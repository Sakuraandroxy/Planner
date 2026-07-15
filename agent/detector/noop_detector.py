"""No-op detector for pure local Qwen closed-loop runs."""

from agent.detector import register_detector
from agent.detector.base import BaseDetector, DetectionResult


@register_detector("none")
@register_detector("noop")
class NoopDetector(BaseDetector):
    """Always returns "target not visible"."""

    def detect(self, image, caption: str, depth_meters=None, camera_name: str = "front") -> DetectionResult:
        return DetectionResult(visible=False, camera=camera_name)
