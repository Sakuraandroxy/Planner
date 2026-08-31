"""Task completion checking independent of trajectory planning."""

from __future__ import annotations

import base64
import io
import json
import re
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from openai import OpenAI
from PIL import ImageDraw

from agent.functions.common.detection_policy import allows_clipped_large_structure
from agent.models.detection.base import DetectionResult


SYSTEM_PROMPT = """You are a deterministic UAV navigation stage completion judge.

You are given one or more current Camera API images with reliable detector evidence.
Each image is identified by camera_id and may have any pitch/yaw/roll; do not infer
its viewing direction from its name or from its position in the message.

Use the provided image evidence, the exact stage target, the overlaid detector box, target depth, and the estimated world distance together. The distance trigger has already stopped the drone near the cached target; your job is to verify target identity and the requested relation.

Your job is to decide:
1. whether the requested target is actually visible in the images, and
2. whether the requested spatial relation is complete now.

Rules:
- Apply the same rule consistently. Do not become more permissive or stricter across repeated calls.
- Do not choose a view just because its detector confidence is higher; verify that the boxed object matches the requested target, including color and object type.
- If a detector box covers most of the image or does not visually match the requested object/color/shape, distrust that proposal.
- If the instruction says above / over / on top, use an image whose supplied optical axis points downward. A target in a horizontally facing image only means the drone is near or facing it.
- For above / over / on top, mark done=true only when the target's main body is clearly visible near the central area of a downward-facing image and that image's depth is within the arrival radius.
- For above / over / on top, if a downward-facing image shows only a small edge/corner/partial fragment of the target, or the target is mainly at the image border, return done=false even if the depth is small.
- For fly to / beside / near, return done=true only when the requested target is visually matched in at least one reliable boxed view AND that same fresh view's target depth is within the arrival radius.
- Estimated world distance is trigger/context information only. It must never replace a fresh boxed target depth or make an otherwise invisible target complete.
- A close target may be large or partially clipped because the drone is already beside it. Cropping alone is NOT a reason for done=false when target identity is clear and distance is within the radius.
- Return done=false only when the requested target identity is not supported by either view, or when every available reliable distance is outside the arrival radius.
- If multiple reliable views are provided and their depths strongly disagree, prefer the view whose box visually matches the requested target and is not a huge or border-clipped proposal.
- If the provided image evidence does not truly show the requested target, return target_detected=false, done=false, accepted_view="none".

Return only one JSON object. Keep reason_code short and deterministic.
Required keys:
{"target_detected": true or false, "done": true or false, "accepted_view": "one supplied camera_id" or "none", "reason_code": "complete_near_target" or "target_mismatch" or "outside_radius"}
"""


USER_PROMPT = """Stage mode: {mode}
Stage instruction: {instruction}
Target: {target}
Relation: {relation}
Completion condition: {completion_condition}
Arrival radius: {arrival_radius_m} meters
Estimated UAV-to-target world distance: {estimated_distance_m}

Detector evidence:
{evidence}

Allowed accepted_view values: {allowed_views}

Question: Looking at the provided image evidence and the task target, is this stage complete now?
Return only {{"target_detected": true or false, "done": true or false, "accepted_view": "one allowed camera_id" or "none", "reason_code": "complete_near_target" or "target_mismatch" or "outside_radius"}}."""


@dataclass
class CompletionResult:
    checked: bool = False
    done: bool = False
    target_detected: Optional[bool] = None
    accepted_view: str = "none"
    reason: str = ""
    detection: Optional[DetectionResult] = None
    direction: str = ""
    elapsed: float = 0.0


class TaskCompletionChecker:
    """Owns detector-guided task completion decisions."""

    name = "depth_detector"
    uses_detector = True

    def __init__(self, cfg: dict, detector=None, direction_estimator=None):
        tc = {**(cfg.get("TASK_COMPLETION", {}) or {}), **cfg.get("FUNCTIONS", {}).get("COMPLETION", {})}
        ag = cfg.get("AGENT", {})
        self.enabled = bool(tc.get("ENABLED", True))
        self.require_depth = bool(tc.get("REQUIRE_DEPTH_FOR_DETECTION", True))
        self.rgb_profile = str(tc.get(
            "PLANNING_CAPTURE_PROFILE",
            tc.get("RGB_CAPTURE_PROFILE", "front_down"),
        ))
        self.depth_profile = str(tc.get("DEPTH_CAPTURE_PROFILE", "front_down_front_depth"))
        self.stop_depth = float(tc.get("STOP_DEPTH_THRESHOLD", ag.get("STOP_DEPTH_THRESHOLD", 8.0)))
        self.detector_name = str(ag.get("DETECTOR", "")).strip().lower()
        self.detector = detector
        self.direction_estimator = direction_estimator
        self.vlm_final_judge_enabled = bool(tc.get("VLM_FINAL_JUDGE_ENABLED", True))
        self.vlm_fail_fallback_to_heuristic = bool(tc.get("VLM_FAIL_FALLBACK_TO_HEURISTIC", False))
        self.url = str(tc.get("URL", "") or "").strip()
        self.model = str(tc.get("MODEL", "") or "").strip()
        self.api_key = str(tc.get("API_KEY", "no-key") or "no-key")
        self.timeout = float(tc.get("TIMEOUT", 60))
        self.max_tokens = int(tc.get("MAX_TOKENS", 128))
        self.temperature = float(tc.get("TEMPERATURE", 0.0))
        perception = cfg.get("FUNCTIONS", {}).get("PERCEPTION", {}) or {}
        self.min_detection_confidence = float(tc.get(
            "MIN_DETECTION_CONFIDENCE",
            perception.get("MIN_CONFIDENCE", ag.get("DETECTOR_MIN_CONFIDENCE", 0.0)),
        ))
        self.max_detection_span = float(tc.get("MAX_DETECTION_SPAN", 0.90))

    def is_detector_enabled(self) -> bool:
        return self.detector is not None and self.detector_name not in {"", "none", "noop"}

    def should_check_stage(self, stage: Any) -> bool:
        return bool(self.enabled and stage and getattr(stage, "mode", "") in {"target", "detect"})

    def capture_profile_for_stage(self, stage: Any) -> str:
        if self.should_check_stage(stage) and self.is_detector_enabled():
            return self.depth_profile
        return self.rgb_profile

    def evaluate(
        self,
        stage: Any,
        fallback_instruction: str,
        front_image,
        down_image,
        front_depth_meters=None,
        down_depth_meters=None,
    ) -> CompletionResult:
        started = time.perf_counter()
        if not self.should_check_stage(stage):
            return CompletionResult(reason="completion disabled or non-target stage")
        if not self.is_detector_enabled():
            return CompletionResult(reason="detector disabled")
        if (
            getattr(stage, "mode", "") == "target"
            and self.require_depth
            and front_depth_meters is None
            and down_depth_meters is None
        ):
            return CompletionResult(reason="depth unavailable; detector skipped")

        caption = (
            getattr(stage, "target_query", None)
            or getattr(stage, "target", None)
            or fallback_instruction
        )
        front_detection = self.detector.detect(
            front_image,
            caption,
            front_depth_meters,
            camera_name="front",
        ) if front_image is not None else DetectionResult(visible=False, camera="front")
        down_detection = self.detector.detect(
            down_image,
            caption,
            down_depth_meters,
            camera_name="down",
        ) if down_image is not None else DetectionResult(visible=False, camera="down")
        return self._finalize_evaluation(
            stage=stage,
            fallback_instruction=fallback_instruction,
            front_image=front_image,
            down_image=down_image,
            front_detection=front_detection,
            down_detection=down_detection,
            detection=None,
            front_depth_meters=front_depth_meters,
            down_depth_meters=down_depth_meters,
            started=started,
        )

    def evaluate_with_detection(
        self,
        stage: Any,
        fallback_instruction: str,
        front_image,
        down_image,
        detection: Optional[DetectionResult],
        front_detection: Optional[DetectionResult] = None,
        down_detection: Optional[DetectionResult] = None,
        front_depth_meters=None,
        down_depth_meters=None,
        estimated_distance_m: Optional[float] = None,
    ) -> CompletionResult:
        """Check completion using already computed RGB detections."""
        started = time.perf_counter()
        if not self.should_check_stage(stage):
            return CompletionResult(reason="completion disabled or non-target stage")
        if not self.is_detector_enabled():
            return CompletionResult(reason="detector disabled")
        return self._finalize_evaluation(
            stage=stage,
            fallback_instruction=fallback_instruction,
            front_image=front_image,
            down_image=down_image,
            front_detection=front_detection,
            down_detection=down_detection,
            detection=detection,
            front_depth_meters=front_depth_meters,
            down_depth_meters=down_depth_meters,
            estimated_distance_m=estimated_distance_m,
            started=started,
        )

    def _finalize_evaluation(
        self,
        stage: Any,
        fallback_instruction: str,
        front_image,
        down_image,
        front_detection: Optional[DetectionResult],
        down_detection: Optional[DetectionResult],
        detection: Optional[DetectionResult],
        front_depth_meters=None,
        down_depth_meters=None,
        estimated_distance_m: Optional[float] = None,
        started: float = 0.0,
    ) -> CompletionResult:
        # Attach depth to both detections before reliability filtering.
        if front_detection is not None:
            front_detection = self._attach_depth(
                front_detection, front_image, down_image, front_depth_meters, down_depth_meters,
            )
        if down_detection is not None:
            down_detection = self._attach_depth(
                down_detection, front_image, down_image, front_depth_meters, down_depth_meters,
            )

        # If a bbox spans >90 % of its image, zero that detection's score.
        # Score-zero proposals are not sent to the VLM final judge.
        if front_detection and front_detection.visible:
            self._zero_giant_bbox_score(stage, front_detection, front_image)
        if down_detection and down_detection.visible:
            self._zero_giant_bbox_score(stage, down_detection, down_image)

        front_ok = front_detection is not None and front_detection.visible
        down_ok = down_detection is not None and down_detection.visible
        if not front_ok and not down_ok:
            return CompletionResult(
                checked=True, done=False, target_detected=False,
                accepted_view="none", reason="fresh target not detected",
                detection=None, direction="",
                elapsed=time.perf_counter() - started,
            )

        reliable_front = front_detection if self._has_reliable_fresh_evidence(front_detection, front_image, stage) else None
        reliable_down = down_detection if self._has_reliable_fresh_evidence(down_detection, down_image, stage) else None

        # ── VLM final judge (only reliable detector-backed image evidence) ──
        if self._can_use_vlm_final_judge():
            if reliable_front is None and reliable_down is None:
                ok, target_detected, accepted_view, done, reason = (
                    True,
                    False,
                    "none",
                    False,
                    "no reliable target evidence",
                )
            else:
                ok, target_detected, accepted_view, done, reason = self._judge_with_vlm(
                    stage=stage,
                    fallback_instruction=fallback_instruction,
                    front_image=front_image,
                    down_image=down_image,
                    front_detection=reliable_front,
                    down_detection=reliable_down,
                    estimated_distance_m=estimated_distance_m,
                )
            if ok:
                # Validate the VLM's decision against physical constraints
                view_evidence = self._view_evidence_map(
                    front_image,
                    down_image,
                    reliable_front,
                    reliable_down,
                )
                accepted_item = view_evidence.get(str(accepted_view).lower())
                if done and target_detected and accepted_item is None:
                    target_detected = False
                    accepted_view = "none"
                    done = False
                    reason = "not complete"
                elif done and target_detected and accepted_item is not None:
                    accepted_det, accepted_image = accepted_item
                    if accepted_det is None or not accepted_det.visible:
                        target_detected = False
                        accepted_view = "none"
                        done = False
                        reason = "not complete"
                    elif not self._is_depth_inside_arrival_radius(
                        accepted_det, estimated_distance_m=estimated_distance_m,
                    ):
                        done = False
                        reason = "not complete"
                if (
                    self._is_above_stage(stage)
                    and done
                    and (
                        accepted_item is None
                        or not self._camera_points_downward(accepted_item[1])
                    )
                ):
                    done = False
                    reason = "not complete"

                accepted_detection = accepted_item[0] if target_detected and accepted_item is not None else None
                accepted_image = accepted_item[1] if accepted_item is not None else front_image
                direction = (
                    self._estimate_direction(accepted_detection, accepted_image)
                    if target_detected else ""
                )
                return CompletionResult(
                    checked=True,
                    done=done,
                    target_detected=target_detected,
                    accepted_view=accepted_view,
                    reason=reason,
                    detection=accepted_detection,
                    direction=direction,
                    elapsed=time.perf_counter() - started,
                )
            if not self.vlm_fail_fallback_to_heuristic:
                return CompletionResult(
                    checked=True, done=False, target_detected=None,
                    accepted_view="none", reason=reason, detection=None,
                    direction="", elapsed=time.perf_counter() - started,
                )

        # ── Heuristic fallback (VLM disabled or failed with fallback enabled) ──
        for det, image, fallback_view in (
            (front_detection, front_image, "front"),
            (down_detection, down_image, "down"),
        ):
            if det is None or not det.visible:
                continue
            if self._is_above_stage(stage) and not self._camera_points_downward(image):
                continue
            if getattr(stage, "mode", "") == "detect":
                return CompletionResult(
                    checked=True, done=True, target_detected=True,
                    accepted_view=self._camera_id_for_detection(det, image, fallback_view), reason="target detected",
                    detection=det,
                    direction=self._estimate_direction(det, image),
                    elapsed=time.perf_counter() - started,
                )
            if det.depth_median is not None and det.depth_median < self.stop_depth:
                return CompletionResult(
                    checked=True, done=True, target_detected=True,
                    accepted_view=self._camera_id_for_detection(det, image, fallback_view),
                    reason=f"depth={det.depth_median:.1f}m < {self.stop_depth:.1f}m",
                    detection=det,
                    direction=self._estimate_direction(det, image),
                    elapsed=time.perf_counter() - started,
                )
        return CompletionResult(
            checked=True, done=False, target_detected=False,
            accepted_view="none", reason="target not complete",
            direction="", elapsed=time.perf_counter() - started,
        )

    @staticmethod
    def _is_above_stage(stage: Any) -> bool:
        relation = str(getattr(stage, "relation", "") or "").strip().lower()
        instruction = str(getattr(stage, "instruction", "") or "").strip().lower()
        return (
            relation in {"above", "over", "on top", "on top of"}
            or "above" in instruction
            or "on top" in instruction
        )

    def _estimate_direction(self, detection: Optional[DetectionResult], image) -> str:
        if not detection or not detection.visible or not detection.bbox:
            return ""
        if self.direction_estimator and image is not None:
            frame = getattr(detection, "camera_frame", None) or getattr(image, "camera_frame", None)
            yaw = float(getattr(frame, "navigation_yaw_deg", 0.0) or 0.0)
            try:
                return self.direction_estimator.estimate(
                    detection=detection,
                    bbox=detection.bbox,
                    image_size=(image.width, image.height),
                    navigation_yaw_deg=yaw,
                )
            except TypeError:
                return self.direction_estimator.estimate(detection.bbox, 0, (image.width, image.height))
        return ""

    def _attach_depth(
        self,
        detection: DetectionResult,
        front_image,
        down_image=None,
        front_depth_meters=None,
        down_depth_meters=None,
    ) -> DetectionResult:
        if detection.depth_median is not None or not detection.bbox:
            return detection
        front_id = self._camera_id_for_image(front_image, "front")
        down_id = self._camera_id_for_image(down_image, "down")
        detection_id = str(getattr(detection, "camera_id", "") or getattr(detection, "camera", "")).lower()
        use_down = detection_id in {str(down_id).lower(), "down"}
        depth = down_depth_meters if use_down else front_depth_meters
        if depth is None:
            return detection

        image = down_image if use_down and down_image is not None else front_image
        if image is None or not hasattr(image, "width") or not hasattr(image, "height"):
            return detection

        try:
            h, w = depth.shape
            sw = w / float(image.width)
            sh = h / float(image.height)
            bbox = detection.bbox
            db = [
                int(bbox[0] * sw), int(bbox[1] * sh),
                int(bbox[2] * sw), int(bbox[3] * sh),
            ]
            cx = (db[0] + db[2]) // 2
            cy = (db[1] + db[3]) // 2
            if 0 <= cy < h and 0 <= cx < w:
                detection.depth_median = float(depth[cy, cx])
                detection.depth_bbox = db
        except Exception:
            pass
        return detection

    @staticmethod
    def _camera_id_for_image(image, fallback: str) -> str:
        frame = getattr(image, "camera_frame", None) if image is not None else None
        return str(getattr(frame, "camera_id", "") or fallback).strip().lower()

    @classmethod
    def _camera_id_for_detection(cls, detection, image, fallback: str) -> str:
        return str(
            getattr(detection, "camera_id", "")
            or cls._camera_id_for_image(image, fallback)
            or getattr(detection, "camera", "")
            or fallback
        ).strip().lower()

    @staticmethod
    def _camera_points_downward(image) -> bool:
        frame = getattr(image, "camera_frame", None) if image is not None else None
        if frame is None:
            return False
        optical = list(frame.optical_axis_world or [])
        if len(optical) < 3:
            return False
        horizontal = (float(optical[0]) ** 2 + float(optical[1]) ** 2) ** 0.5
        return bool(float(optical[2]) > 0.35 and float(optical[2]) > 0.5 * horizontal)

    @classmethod
    def _view_evidence_map(cls, front_image, down_image, front_detection, down_detection):
        mapping = {
            cls._camera_id_for_image(front_image, "front"): (front_detection, front_image),
            cls._camera_id_for_image(down_image, "down"): (down_detection, down_image),
        }
        # Legacy aliases are input-slot adapters, not orientation assumptions.
        mapping.setdefault("front", (front_detection, front_image))
        mapping.setdefault("down", (down_detection, down_image))
        return mapping

    def _prepare_detection(
        self,
        detection: Optional[DetectionResult],
        view_name: str,
        front_image,
        down_image,
        front_depth_meters=None,
        down_depth_meters=None,
    ) -> Optional[DetectionResult]:
        if detection is None:
            return None
        prepared = self._attach_depth(
            detection,
            front_image,
            down_image,
            front_depth_meters,
            down_depth_meters,
        )
        image = down_image if view_name == "down" else front_image
        if not self._has_reliable_fresh_evidence(prepared, image):
            return None
        return prepared

    def _has_reliable_fresh_evidence(self, detection: Optional[DetectionResult], image, stage: Any = None) -> bool:
        if detection is None or not detection.visible or not detection.bbox:
            return False
        if float(detection.score or 0.0) < self.min_detection_confidence:
            return False
        if image is None or not hasattr(image, "size") or len(detection.bbox) < 4:
            return False
        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return False
        x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
        box_w = max(0.0, min(width, x2) - max(0.0, x1))
        box_h = max(0.0, min(height, y2) - max(0.0, y1))
        if box_w <= 1.0 or box_h <= 1.0:
            return False
        exceeds_span = box_w / width >= self.max_detection_span or box_h / height >= self.max_detection_span
        return bool(
            not exceeds_span
            or allows_clipped_large_structure(
                stage,
                detection,
                image,
                max_span=self.max_detection_span,
            )
        )

    def _select_best_detection(
        self,
        detection: Optional[DetectionResult],
        front_detection: Optional[DetectionResult],
        down_detection: Optional[DetectionResult],
    ) -> Optional[DetectionResult]:
        if detection is not None and detection.visible:
            return detection
        visible = [d for d in (detection, front_detection, down_detection) if d and d.visible]
        if not visible:
            return detection or front_detection or down_detection
        return max(visible, key=lambda d: float(d.score or 0.0))

    @staticmethod
    def _select_detection_by_view(
        accepted_view: str,
        front_detection: Optional[DetectionResult],
        down_detection: Optional[DetectionResult],
        fallback_detection: Optional[DetectionResult] = None,
    ) -> Optional[DetectionResult]:
        view = (accepted_view or "none").strip().lower()
        if view == "front":
            return front_detection
        if view == "down":
            return down_detection
        return fallback_detection

    def _is_depth_inside_arrival_radius(
        self,
        detection: Optional[DetectionResult],
        estimated_distance_m: Optional[float] = None,
    ) -> bool:
        if detection is None or not detection.visible:
            return False
        if detection.depth_median is None:
            return not self.require_depth
        return float(detection.depth_median) <= self.stop_depth

    @staticmethod
    def _zero_giant_bbox_score(stage: Any, detection: DetectionResult, image) -> None:
        """Zero detection.score when the bbox covers >90 % of the image.

        The detection object is kept for logs/debug evidence, but score=0
        makes the VLM request builder skip the corresponding image.
        """
        if detection is None or not detection.bbox or image is None or not hasattr(image, "size"):
            return
        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return
        x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
        box_w = max(0.0, min(width, x2) - max(0.0, x1))
        box_h = max(0.0, min(height, y2) - max(0.0, y1))
        if box_w / width >= 0.90 or box_h / height >= 0.90:
            if allows_clipped_large_structure(stage, detection, image, max_span=0.90):
                return
            detection.score = 0.0

    @staticmethod
    def _bbox_exceeds_image_span(
        detection: Optional[DetectionResult],
        image,
        max_span: float,
    ) -> bool:
        if detection is None or not detection.bbox or image is None or not hasattr(image, "size"):
            return False
        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return False
        x1, y1, x2, y2 = [float(v) for v in detection.bbox[:4]]
        box_w = max(0.0, min(width, x2) - max(0.0, x1))
        box_h = max(0.0, min(height, y2) - max(0.0, y1))
        return (box_w / width) >= max_span or (box_h / height) >= max_span

    def _heuristic_decision(self, stage: Any, detection: Optional[DetectionResult]) -> Tuple[bool, str]:
        if detection is None or not detection.visible:
            return False, "target not visible"
        if getattr(stage, "mode", "") == "detect":
            return True, "target detected"
        if detection.depth_median is not None and detection.depth_median < self.stop_depth:
            return True, f"depth={detection.depth_median:.1f}m < {self.stop_depth:.1f}m"
        if detection.depth_median is None:
            return False, "target visible but depth unavailable"
        return False, "target visible but not complete"

    def _can_use_vlm_final_judge(self) -> bool:
        if not (self.vlm_final_judge_enabled and self.url and self.model):
            return False
        lowered_url = self.url.lower()
        if "xiaomimimo.com" in lowered_url and self.api_key.strip().lower() in {"", "no-key"}:
            return False
        return True

    def _judge_with_vlm(
        self,
        stage: Any,
        fallback_instruction: str,
        front_image,
        down_image,
        front_detection: Optional[DetectionResult],
        down_detection: Optional[DetectionResult],
        estimated_distance_m: Optional[float] = None,
    ) -> Tuple[bool, bool, str, bool, str]:
        if front_image is None and down_image is None:
            return False, False, "none", False, "vlm judge skipped: camera images unavailable"

        instruction = (getattr(stage, "instruction", "") or fallback_instruction or "").strip()
        target = (getattr(stage, "target_query", "") or getattr(stage, "target", "") or "").strip()
        relation = (getattr(stage, "relation", "") or "").strip()
        completion_condition = (getattr(stage, "completion_condition", "") or "").strip()
        mode = (getattr(stage, "mode", "") or "").strip()
        front_view_id = self._camera_id_for_image(front_image, "front")
        down_view_id = self._camera_id_for_image(down_image, "down")
        prefer_down_view = self._is_above_stage(stage) and self._camera_points_downward(down_image)
        evidence_items = []
        if front_detection is not None:
            evidence_items.append(self._format_detection_evidence(front_view_id, front_detection, front_image))
        if down_detection is not None:
            evidence_items.append(self._format_detection_evidence(down_view_id, down_detection, down_image))
        if not evidence_items:
            return False, False, "none", False, "vlm judge skipped: no reliable image evidence"
        if prefer_down_view:
            evidence_items = sorted(
                evidence_items,
                key=lambda item: 0 if item.startswith(f"- {down_view_id}:") else 1,
            )
        evidence = "\n".join(evidence_items)
        user_text = USER_PROMPT.format(
            mode=mode or "unknown",
            instruction=instruction,
            target=target,
            relation=relation or "none",
            completion_condition=completion_condition or "none",
            arrival_radius_m=f"{self.stop_depth:.2f}",
            estimated_distance_m=(
                f"{float(estimated_distance_m):.2f} meters"
                if estimated_distance_m is not None
                else "unavailable"
            ),
            evidence=evidence,
            allowed_views=", ".join(dict.fromkeys([front_view_id, down_view_id, "none"])),
        )
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {
                "role": "user",
                "content": self._build_user_content(
                    user_text,
                    front_image,
                    down_image,
                    front_detection=front_detection,
                    down_detection=down_detection,
                    prefer_down_view=prefer_down_view,
                ),
            },
        ]

        try:
            client = OpenAI(base_url=self.url, api_key=self.api_key, timeout=self.timeout)
            resp = client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                extra_body={"thinking": {"type": "disabled"}},
            )
            raw = resp.choices[0].message.content or ""
            data = self._parse_response(raw)
            target_detected = bool(data.get("target_detected", False))
            accepted_view = str(data.get("accepted_view", "none") or "none").strip().lower()
            if accepted_view == "front":
                accepted_view = front_view_id
            elif accepted_view == "down":
                accepted_view = down_view_id
            allowed_views = {front_view_id, down_view_id, "none"}
            if accepted_view not in allowed_views:
                accepted_view = "none"
            if not target_detected:
                accepted_view = "none"
            done = bool(data.get("done", False)) if target_detected else False
            reason_code = str(data.get("reason_code", "") or "").strip().lower()
            reason = reason_code or ("complete" if done else "not complete")
            return True, target_detected, accepted_view, done, reason
        except Exception as exc:
            return False, False, "none", False, f"vlm judge failed: {exc}"

    def _build_user_content(
        self,
        text: str,
        front_image,
        down_image,
        front_detection: Optional[DetectionResult] = None,
        down_detection: Optional[DetectionResult] = None,
        prefer_down_view: bool = False,
    ):
        content = [{"type": "text", "text": text}]

        def has_reliable_detection(detection: Optional[DetectionResult]) -> bool:
            return bool(
                detection is not None
                and detection.visible
                and detection.bbox
                and float(detection.score or 0.0) >= self.min_detection_confidence
            )

        def append_front():
            if front_image is None or not has_reliable_detection(front_detection):
                return
            view_id = self._camera_id_for_image(front_image, "front")
            content.append({"type": "text", "text": f"Camera {view_id} image with proposed detection box overlay:"})
            content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{self._to_b64(self._overlay_detection(front_image, front_detection, view_id))}"
                },
            })

        def append_down():
            if down_image is None or not has_reliable_detection(down_detection):
                return
            view_id = self._camera_id_for_image(down_image, "down")
            content.append({"type": "text", "text": f"Camera {view_id} image with proposed detection box overlay:"})
            content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{self._to_b64(self._overlay_detection(down_image, down_detection, view_id))}"
                },
            })

        if prefer_down_view:
            append_down()
            append_front()
        else:
            append_front()
            append_down()
        return content

    @staticmethod
    def _overlay_detection(image, detection: Optional[DetectionResult], view_name: str):
        if image is None:
            return image
        canvas = image.convert("RGB").copy()
        draw = ImageDraw.Draw(canvas)
        if detection is None or not detection.bbox:
            draw.text((12, 12), f"{view_name}: no proposal", fill=(255, 255, 0))
            return canvas

        x1, y1, x2, y2 = [int(v) for v in detection.bbox]
        color = (255, 64, 64) if "front" in str(view_name).lower() else (64, 200, 255)
        width = max(3, int(round(min(canvas.size) / 256)))
        for offset in range(width):
            draw.rectangle(
                [x1 - offset, y1 - offset, x2 + offset, y2 + offset],
                outline=color,
            )
        label = f"{view_name} score={float(detection.score or 0.0):.2f}"
        if detection.depth_median is not None:
            label += f" depth={float(detection.depth_median):.1f}m"
        text_y = max(8, y1 - 24)
        draw.rectangle([x1, text_y, min(canvas.size[0] - 1, x1 + 260), text_y + 18], fill=(0, 0, 0))
        draw.text((x1 + 4, text_y + 2), label, fill=color)
        return canvas

    @staticmethod
    def _to_b64(img) -> str:
        if img.mode == "RGBA":
            img = img.convert("RGB")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=90)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    @staticmethod
    def _parse_response(raw: str) -> Dict[str, Any]:
        text = (raw or "").strip()
        text = re.sub(r"```json\s*", "", text, flags=re.IGNORECASE)
        text = re.sub(r"```\s*", "", text)
        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            data = json.loads(text[start:end + 1])
            if "target_detected" not in data:
                data["target_detected"] = bool(data.get("done", False))
            if "accepted_view" not in data:
                data["accepted_view"] = "front" if data.get("target_detected") else "none"
            return data
        lowered = text.lower()
        if "target_detected" in lowered:
            detected = '"target_detected": true' in lowered or "'target_detected': true" in lowered
            done = '"done": true' in lowered or "'done': true" in lowered
            accepted_view = "none"
            if '"accepted_view": "down"' in lowered or "'accepted_view': 'down'" in lowered:
                accepted_view = "down"
            elif '"accepted_view": "front"' in lowered or "'accepted_view': 'front'" in lowered:
                accepted_view = "front"
            elif detected:
                accepted_view = "front"
            return {"target_detected": detected, "done": detected and done, "accepted_view": accepted_view}
        if "true" in lowered and "false" not in lowered:
            return {"target_detected": True, "done": True, "accepted_view": "front"}
        if "false" in lowered:
            return {"target_detected": False, "done": False, "accepted_view": "none"}
        return {"target_detected": False, "done": False, "accepted_view": "none"}

    @staticmethod
    def _format_detection_evidence(view_name: str, detection: Optional[DetectionResult], image) -> str:
        image_size = "unknown"
        area_ratio = "unknown"
        center = "unknown"
        if image is not None and hasattr(image, "size"):
            image_size = f"{image.size[0]}x{image.size[1]}"
        if detection is None:
            return f"- {view_name}: image_size={image_size}, visible=false"
        bbox = detection.bbox if detection.bbox else None
        if bbox and image is not None and hasattr(image, "size"):
            width, height = float(image.size[0]), float(image.size[1])
            box_w = max(0.0, float(bbox[2] - bbox[0]))
            box_h = max(0.0, float(bbox[3] - bbox[1]))
            denom = max(width * height, 1.0)
            area_ratio = f"{(box_w * box_h / denom):.3f}"
            center = f"({(bbox[0] + bbox[2]) / (2.0 * max(width, 1.0)):.3f}, {(bbox[1] + bbox[3]) / (2.0 * max(height, 1.0)):.3f})"
        depth = f"{detection.depth_median:.2f}m" if detection.depth_median is not None else "unknown"
        label = detection.label or ""
        frame = getattr(detection, "camera_frame", None) or getattr(image, "camera_frame", None)
        optical_axis = (
            "[" + ",".join(f"{float(value):.3f}" for value in frame.optical_axis_world[:3]) + "]"
            if frame is not None
            else "unknown"
        )
        return (
            f"- {view_name}: image_size={image_size}, visible={bool(detection.visible)}, "
            f"bbox_px={bbox}, score={float(detection.score or 0.0):.2f}, "
            f"depth={depth}, optical_axis_world={optical_axis}, "
            f"bbox_area_ratio={area_ratio}, bbox_center_norm={center}, label={label}"
        )
