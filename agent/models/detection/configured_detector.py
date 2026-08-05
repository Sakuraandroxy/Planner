"""Model wrapper for the detector selected in config/default.yaml."""

from __future__ import annotations

from agent.core.types import Detection
from agent.models.detection import build_detector
from agent.models.detection.model_base import DetectionModel


class ConfiguredDetectorModel(DetectionModel):
    """Expose the configured detector through the shared model contract."""

    def __init__(self, config=None, detector=None):
        self.config = config or {}
        self.detector = detector or build_detector()

    def detect(self, image, text: str, *, view: str = "front", depth_meters=None) -> Detection:
        result = self.detector.detect(image, text, depth_meters=depth_meters, camera_name=view)
        return Detection(
            visible=bool(result.visible),
            view=getattr(result, "camera", view) or view,
            bbox=list(result.bbox) if result.bbox else None,
            score=float(result.score or 0.0),
            label=str(result.label or text or ""),
            depth_median=result.depth_median,
            surface_depth_samples=(
                [list(sample) for sample in result.surface_depth_samples]
                if getattr(result, "surface_depth_samples", None)
                else None
            ),
            source=self.detector.__class__.__name__,
            raw=result,
        )
