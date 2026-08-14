"""Four-direction target relocalization for detect/target stages."""

from __future__ import annotations

import contextlib
import io
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from agent.functions.common.detection_policy import detection_caption_for_stage, detection_reliability
from agent.models.detection.base import DetectionResult


@dataclass
class RelocalizationResult:
    found: bool = False
    detection: Optional[DetectionResult] = None
    front_image: Any = None
    down_image: Any = None
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
        self.accept_down_view = bool(rcfg.get("ACCEPT_DOWN_VIEW", False))
        self.center_on_target = bool(rcfg.get("CENTER_ON_TARGET", True))
        self.camera_hfov_deg = float(rcfg.get("CAMERA_HFOV_DEG", 90.0))
        self.min_center_offset_deg = float(rcfg.get("MIN_CENTER_OFFSET_DEG", 3.0))
        self.min_bbox_area_ratio = float(rcfg.get("MIN_BBOX_AREA_RATIO", 0.002))
        self.max_bbox_span = float(rcfg.get("MAX_BBOX_SPAN", 0.90))
        self.detector = detector
        ag = cfg.get("AGENT", {})
        perception = cfg.get("FUNCTIONS", {}).get("PERCEPTION", {}) or {}
        self.min_confidence = float(rcfg.get(
            "MIN_CONFIDENCE",
            perception.get(
                "MIN_CONFIDENCE",
                ag.get("DETECTOR_MIN_CONFIDENCE", ag.get("DETECTOR_BOX_THRESHOLD", 0.0)),
            ),
        ))

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

        target = detection_caption_for_stage(stage)
        if not target:
            return RelocalizationResult(reason="empty target")

        _start_pos, start_yaw = client.get_pose()
        turns_done = 0
        for idx in range(max(1, self.max_turns)):
            if idx == 0 and front_image is not None and not skip_initial_frame:
                frame, down = front_image, down_image
            else:
                if skip_initial_frame or idx > 0:
                    client.rotate_yaw(self.turn_deg)
                    turns_done += 1
                frame, down, _fd, _dd, _timing = client.capture_views(
                    profile=self.capture_profile,
                    mode=capture_mode,
                    verbose=False,
                )

            front_det, down_det, detection = self._detect_views(stage, frame, down, target)
            accepted = None
            if validator is not None:
                try:
                    accepted = validator(stage, frame, down, front_det, down_det, detection)
                except Exception:
                    accepted = None
            elif detection.visible:
                accepted = detection

            if accepted is not None and accepted.visible:
                center_offset = 0.0
                if accepted.camera == "front" and self.center_on_target:
                    center_offset = self._front_center_offset_deg(accepted, frame)
                    if abs(center_offset) >= self.min_center_offset_deg:
                        client.rotate_yaw(center_offset)
                return RelocalizationResult(
                    found=True,
                    detection=accepted,
                    front_image=frame,
                    down_image=down,
                    direction_index=idx,
                    yaw_delta_deg=(turns_done * self.turn_deg) + center_offset,
                    elapsed=time.perf_counter() - started,
                    reason=f"target detected in {accepted.camera} view",
                )

        # Do not leave the vehicle facing an arbitrary scan direction after a
        # failed search. Restore the exact heading from before relocalization.
        client.rotate_to_yaw(start_yaw)
        return RelocalizationResult(
            found=False,
            elapsed=time.perf_counter() - started,
            reason=f"'{target}' not found after {self.max_turns} views",
        )

    def _detect_views(self, stage: Any, front_image, down_image, target: str):
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
        candidates = (front_det, down_det) if self.accept_down_view else (front_det,)
        visible = [
            d for d in candidates
            if d and d.visible and self._bbox_is_reliable(
                stage,
                d,
                front_image if d.camera == "front" else down_image,
            )
        ]
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

    def _bbox_is_reliable(self, stage: Any, detection: DetectionResult, image) -> bool:
        if not detection.bbox or image is None or not hasattr(image, "size"):
            return False
        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return False
        x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
        box_w = max(0.0, min(width, x2) - max(0.0, x1))
        box_h = max(0.0, min(height, y2) - max(0.0, y1))
        area_ratio = box_w * box_h / max(width * height, 1.0)
        return bool(
            area_ratio >= self.min_bbox_area_ratio
            and detection_reliability(
                stage,
                detection,
                image,
                max_span=self.max_bbox_span,
            ) > 0.0
        )

    def _front_center_offset_deg(self, detection: DetectionResult, image) -> float:
        if not detection.bbox or image is None or not hasattr(image, "width") or image.width <= 1:
            return 0.0
        center_x = (float(detection.bbox[0]) + float(detection.bbox[2])) / 2.0
        normalized_x = center_x / float(image.width) - 0.5
        return normalized_x * self.camera_hfov_deg

