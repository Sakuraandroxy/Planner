"""Four-direction target relocalization for detect/target stages."""

from __future__ import annotations

import contextlib
import io
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from agent.detector.base import DetectionResult


@dataclass
class RelocalizationResult:
    found: bool = False
    detection: Optional[DetectionResult] = None
    direction_index: int = 0
    yaw_delta_deg: float = 0.0
    elapsed: float = 0.0
    reason: str = ""


class TargetRelocalizer:
    """Search a target with front/down views, rotating in-place if needed."""

    def __init__(self, cfg: dict, detector=None):
        rcfg = cfg.get("RELOCALIZATION", {})
        self.enabled = bool(rcfg.get("ENABLED", True))
        self.turn_deg = float(rcfg.get("TURN_DEG", 90.0))
        self.max_turns = int(rcfg.get("MAX_TURNS", 4))
        self.capture_profile = str(rcfg.get("CAPTURE_PROFILE", "front_down"))
        self.detector = detector
        ag = cfg.get("AGENT", {})
        self.min_confidence = float(ag.get("DETECTOR_MIN_CONFIDENCE", ag.get("DETECTOR_BOX_THRESHOLD", 0.0)))

    def search(
        self,
        client,
        stage: Any,
        front_image=None,
        down_image=None,
        capture_mode: str = "parallel",
        skip_initial_frame: bool = False,
        validator: Optional[Callable[[Any, Any, Any, DetectionResult, DetectionResult, DetectionResult], Optional[DetectionResult]]] = None,
    ) -> RelocalizationResult:
        started = time.perf_counter()
        if not self.enabled:
            return RelocalizationResult(reason="relocalization disabled")
        if self.detector is None:
            return RelocalizationResult(reason="detector unavailable")

        target = (
            getattr(stage, "target_query", None)
            or getattr(stage, "target", None)
            or getattr(stage, "instruction", "")
        )
        if not target:
            return RelocalizationResult(reason="empty target")

        for idx in range(max(1, self.max_turns)):
            yaw_delta_deg = idx * self.turn_deg
            if idx == 0 and front_image is not None and not skip_initial_frame:
                frame, down = front_image, down_image
            else:
                if idx == 0 and skip_initial_frame:
                    client.rotate_yaw(self.turn_deg)
                    yaw_delta_deg = self.turn_deg
                frame, down, _fd, _dd, _timing = client.capture_views(
                    profile=self.capture_profile,
                    mode=capture_mode,
                    verbose=False,
                )

            front_det, down_det, detection = self._detect_views(frame, down, target)
            accepted = None
            if validator is not None:
                try:
                    accepted = validator(stage, frame, down, front_det, down_det, detection)
                except Exception:
                    accepted = None
            elif detection.visible:
                accepted = detection

            if accepted is not None and accepted.visible:
                return RelocalizationResult(
                    found=True,
                    detection=accepted,
                    direction_index=idx,
                    yaw_delta_deg=yaw_delta_deg,
                    elapsed=time.perf_counter() - started,
                    reason=f"target detected in {accepted.camera} view",
                )

            if idx < self.max_turns - 1:
                client.rotate_yaw(self.turn_deg)

        return RelocalizationResult(
            found=False,
            elapsed=time.perf_counter() - started,
            reason=f"'{target}' not found after {self.max_turns} views",
        )

    def _detect_views(self, front_image, down_image, target: str):
        if front_image is None:
            empty = DetectionResult(visible=False, camera="none")
            return empty, empty, empty
        with contextlib.redirect_stdout(io.StringIO()):
            front_det = self.detector.detect(
                front_image,
                target,
                depth_meters=None,
                camera_name="front",
            )
            down_det = (
                self.detector.detect(
                    down_image,
                    target,
                    depth_meters=None,
                    camera_name="down",
                ) if down_image is not None else DetectionResult(visible=False, camera="down")
            )
        visible = [d for d in (front_det, down_det) if d and d.visible]
        if not visible:
            return front_det, down_det, DetectionResult(visible=False, camera="none")
        best = max(visible, key=lambda d: float(d.score or 0.0))
        if float(best.score or 0.0) < self.min_confidence:
            return front_det, down_det, DetectionResult(
                visible=False,
                bbox=best.bbox,
                score=float(best.score or 0.0),
                label=best.label,
                camera=best.camera,
            )
        return front_det, down_det, best
