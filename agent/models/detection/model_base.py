"""Base interface for target-box model backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from agent.core.types import Detection


class DetectionModel(ABC):
    @abstractmethod
    def detect(self, image, text: str, *, view: str = "front", depth_meters=None) -> Detection:
        """Return the best target box for one image."""
        ...

    def detect_all(self, image, text: str, *, view: str = "front", depth_meters=None) -> List[Detection]:
        det = self.detect(image, text, view=view, depth_meters=depth_meters)
        return [det] if det.visible else []
