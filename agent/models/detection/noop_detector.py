"""No-op detector for pure local Qwen closed-loop runs."""

from agent.models.detection import register_detector
from agent.models.detection.base import BaseDetector, DetectionResult


@register_detector("none")
@register_detector("noop")
class NoopDetector(BaseDetector):
    """Always returns "target not visible"."""

    def detect(self, image, caption: str, depth_meters=None, camera_name: str = "front") -> DetectionResult:
        return DetectionResult(visible=False, camera=camera_name)

