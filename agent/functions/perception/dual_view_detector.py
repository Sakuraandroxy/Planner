"""Dual-view target detection function."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import Optional

from agent.core.types import Detection, ImageBundle
from agent.models.detection.configured_detector import ConfiguredDetectorModel


class DualViewDetector:
    """Run a detection model on front/down views and choose the best output."""

    def __init__(self, config=None, model=None):
        self.config = config or {}
        self.model = model or ConfiguredDetectorModel()
        self.min_confidence = float(self.config.get("MIN_CONFIDENCE", 0.0) or 0.0)
        self.parallel = bool(self.config.get("PARALLEL", True))

    def detect(self, images: ImageBundle, text: str) -> Detection:
        def _front():
            if images.front_rgb is None:
                return Detection(False, view="front")
            return self.model.detect(
                images.front_rgb,
                text,
                view="front",
                depth_meters=images.front_depth,
            )

        def _down():
            if images.down_rgb is None:
                return Detection(False, view="down")
            return self.model.detect(
                images.down_rgb,
                text,
                view="down",
                depth_meters=images.down_depth,
            )

        if self.parallel and images.down_rgb is not None:
            with ThreadPoolExecutor(max_workers=2) as executor:
                front = executor.submit(_front)
                down = executor.submit(_down)
                detections = [front.result(), down.result()]
        else:
            detections = [_front(), _down()]

        visible = [d for d in detections if d.visible]
        if not visible:
            return Detection(False, view="none")
        best = max(visible, key=lambda d: float(d.score or 0.0))
        if best.score < self.min_confidence:
            return Detection(False, view=best.view, bbox=best.bbox, score=best.score, raw=best.raw)
        return best
