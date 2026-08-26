"""Detection model base classes and result types."""

from __future__ import annotations

from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class DetectionResult:
    """Unified detection result."""

    visible: bool
    bbox: Optional[List[int]] = None
    score: float = 0.0
    label: str = ""
    depth_median: Optional[float] = None
    depth_bbox: Optional[List[int]] = None
    # Depth quality is kept separate from visibility. A target can provide a
    # reliable RGB bearing even when its metric depth is too sparse or noisy.
    depth_valid_ratio: Optional[float] = None
    depth_mad_m: Optional[float] = None
    # Diagnostics for detecting a bbox that spans foreground/background
    # entities. ``depth_median`` is measured in the central crop, while
    # ``depth_bbox_median`` covers the entire detector box.
    depth_bbox_median: Optional[float] = None
    depth_p10_m: Optional[float] = None
    depth_p90_m: Optional[float] = None
    depth_sample_count: int = 0
    # Normalized image coordinates plus radial depth: [u, v, depth_m].
    # Populated when a completion/memory depth frame is attached.
    surface_depth_samples: Optional[List[List[float]]] = None
    # Large facades may contain windows/background and therefore fail the
    # whole-box metric-depth gate.  Memory can still use a coherent foreground
    # surface patch without pretending that the entire bbox has one depth.
    surface_anchor_uv: Optional[List[float]] = None
    surface_lock_fallback: bool = False
    surface_lock_depth_reason: str = ""
    camera: str = "front"


class BaseDetector(ABC):
    """Detector backend interface."""

    @abstractmethod
    def detect(self, image, caption: str, depth_meters=None, camera_name: str = "front") -> DetectionResult:
        ...

    def detect_all(self, image, caption: str, depth_meters=None, camera_name: str = "front") -> List[DetectionResult]:
        result = self.detect(image, caption, depth_meters, camera_name=camera_name)
        return [result] if result.visible else []

    def detect_with_fallback(
        self,
        front_image,
        down_image,
        caption: str,
        front_depth_meters=None,
        down_depth_meters=None,
    ) -> DetectionResult:
        front_result = self.detect(front_image, caption, front_depth_meters, camera_name="front")
        if front_result.visible:
            front_result.camera = "front"
            return front_result

        if down_image is not None:
            down_result = self.detect(down_image, caption, down_depth_meters, camera_name="down")
            if down_result.visible:
                down_result.camera = "down"
                return down_result

        return DetectionResult(visible=False, camera="none")

    def detect_best_dual_view(
        self,
        front_image,
        down_image,
        caption: str,
        front_depth_meters=None,
        down_depth_meters=None,
        min_confidence: float = 0.0,
        parallel: bool = True,
    ) -> DetectionResult:
        if front_image is None:
            return DetectionResult(visible=False, camera="none")

        def detect_front():
            return self.detect(front_image, caption, front_depth_meters, camera_name="front")

        def detect_down():
            if down_image is None:
                return DetectionResult(visible=False, camera="down")
            return self.detect(down_image, caption, down_depth_meters, camera_name="down")

        if parallel and down_image is not None:
            with ThreadPoolExecutor(max_workers=2) as executor:
                results = [executor.submit(detect_front).result(), executor.submit(detect_down).result()]
        else:
            results = [detect_front(), detect_down()]

        visible = [r for r in results if r and r.visible]
        if not visible:
            return DetectionResult(visible=False, camera="none")

        best = max(visible, key=lambda r: float(r.score or 0.0))
        if float(best.score or 0.0) < float(min_confidence):
            return DetectionResult(
                visible=False,
                bbox=best.bbox,
                score=float(best.score or 0.0),
                label=best.label,
                camera=best.camera,
            )
        return best
