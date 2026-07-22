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

from agent.models.detection.base import DetectionResult


SYSTEM_PROMPT = """You are a UAV navigation stage completion judge.

You are given two current camera images:
- front-view: what the drone sees ahead
- down-view: what the drone sees below

Judge the current stage like a human operator would: look at BOTH images and the stage instruction first. Detector boxes, confidences, and depth values are only auxiliary hints; they may be wrong.

Your job is to decide:
1. whether the requested target is actually visible in the images, and
2. whether the requested spatial relation is complete now.

Rules:
- Do not choose a view just because its detector confidence is higher.
- If a detector box covers most of the image or does not visually match the requested object/color/shape, distrust that proposal.
- If the instruction says above / over / on top, actively inspect the down-view. A front-view target only means the drone is near or facing it; it does not by itself mean the drone is above it.
- For above / over / on top, mark done=true only when the target's main body is clearly visible near the central area of the down-view and the down-view depth is within the arrival radius.
- For above / over / on top, if the down-view shows only a small edge/corner/partial fragment of the target, or the target is mainly at the image border, return done=false even if the depth is small.
- For fly to / beside / near, use the front-view or down-view that best matches the actual target and judge distance with the corresponding depth.
- For fly to / beside / near, target visibility alone is not completion. If the target is only a clipped edge/corner in down-view, mostly outside the image, or represented by a box covering most of the image, return done=false.
- If front-view depth and down-view depth strongly disagree, prefer the view whose box visually matches the requested target and is not a huge or border-clipped proposal.
- If neither image truly shows the requested target, return target_detected=false, done=false, accepted_view="none".

Return only one JSON object. No reasoning. No explanation.
Required keys:
{"target_detected": true or false, "done": true or false, "accepted_view": "front" or "down" or "none"}
"""


USER_PROMPT = """Stage mode: {mode}
Stage instruction: {instruction}
Target: {target}
Relation: {relation}
Completion condition: {completion_condition}
Arrival radius: {arrival_radius_m} meters

Detector evidence:
{evidence}

Question: Looking at both images and the task target, is this stage complete now?
Return only {{"target_detected": true or false, "done": true or false, "accepted_view": "front" or "down" or "none"}}.
No reason. No explanation."""


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
        started: float = 0.0,
    ) -> CompletionResult:
        front_detection = self._prepare_detection(
            front_detection,
            "front",
            front_image,
            down_image,
            front_depth_meters,
            down_depth_meters,
        )
        down_detection = self._prepare_detection(
            down_detection,
            "down",
            front_image,
            down_image,
            front_depth_meters,
            down_depth_meters,
        )
        best_detection = self._select_best_detection(
            detection=detection,
            front_detection=front_detection,
            down_detection=down_detection,
        )

        direction = self._estimate_direction(best_detection, front_image)
        heuristic_done, heuristic_reason = self._heuristic_decision(stage, best_detection)

        if self._can_use_vlm_final_judge():
            ok, target_detected, accepted_view, done, reason = self._judge_with_vlm(
                stage=stage,
                fallback_instruction=fallback_instruction,
                front_image=front_image,
                down_image=down_image,
                front_detection=front_detection,
                down_detection=down_detection,
            )
            if ok:
                accepted_detection = self._select_detection_by_view(
                    accepted_view,
                    front_detection=front_detection,
                    down_detection=down_detection,
                    fallback_detection=best_detection,
                ) if target_detected else None
                if target_detected and accepted_view in {"front", "down"}:
                    if accepted_detection is None or not accepted_detection.visible:
                        target_detected = False
                        accepted_view = "none"
                        accepted_detection = None
                        done = False
                        reason = "not complete"
                    elif done and not self._is_depth_inside_arrival_radius(accepted_detection):
                        done = False
                        reason = "not complete"
                accepted_direction = self._estimate_direction(accepted_detection, front_image) if target_detected else ""
                if self._is_above_stage(stage) and done:
                    done = bool(
                        accepted_view == "down"
                        and accepted_detection is not None
                        and accepted_detection.depth_median is not None
                        and accepted_detection.depth_median < self.stop_depth
                    )
                    reason = "complete" if done else "not complete"
                return CompletionResult(
                    checked=True,
                    done=done,
                    target_detected=target_detected,
                    accepted_view=accepted_view,
                    reason=reason,
                    detection=accepted_detection,
                    direction=accepted_direction,
                    elapsed=time.perf_counter() - started,
                )
            if not self.vlm_fail_fallback_to_heuristic:
                return CompletionResult(
                    checked=True,
                    done=False,
                    target_detected=None,
                    accepted_view="none",
                    reason=reason,
                    detection=best_detection,
                    direction=direction,
                    elapsed=time.perf_counter() - started,
                )

        return CompletionResult(
            checked=True,
            done=heuristic_done,
            target_detected=bool(best_detection and best_detection.visible),
            accepted_view=(best_detection.camera if best_detection and best_detection.visible else "none"),
            reason=heuristic_reason,
            detection=best_detection,
            direction=direction,
            elapsed=time.perf_counter() - started,
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

    def _estimate_direction(self, detection: Optional[DetectionResult], front_image) -> str:
        if not detection or not detection.visible or not detection.bbox:
            return ""
        if detection.camera == "down":
            return "Target is visible in the downward view below the drone."
        if detection.camera == "front" and self.direction_estimator and front_image is not None:
            return self.direction_estimator.estimate(detection.bbox, 0, (front_image.width, front_image.height))
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
        depth = down_depth_meters if detection.camera == "down" else front_depth_meters
        if depth is None:
            return detection

        image = down_image if detection.camera == "down" and down_image is not None else front_image
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
        image = down_image if view_name == "down" and down_image is not None else front_image
        if self._bbox_exceeds_image_span(detection, image, max_span=0.90):
            return None
        return self._attach_depth(
            detection,
            front_image,
            down_image,
            front_depth_meters,
            down_depth_meters,
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

    def _is_depth_inside_arrival_radius(self, detection: Optional[DetectionResult]) -> bool:
        if detection is None or not detection.visible:
            return False
        if detection.depth_median is None:
            return not self.require_depth
        return float(detection.depth_median) < self.stop_depth

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
    ) -> Tuple[bool, bool, str, bool, str]:
        if front_image is None:
            return False, False, "none", False, "vlm judge skipped: front image unavailable"

        instruction = (getattr(stage, "instruction", "") or fallback_instruction or "").strip()
        target = (getattr(stage, "target_query", "") or getattr(stage, "target", "") or "").strip()
        relation = (getattr(stage, "relation", "") or "").strip()
        completion_condition = (getattr(stage, "completion_condition", "") or "").strip()
        mode = (getattr(stage, "mode", "") or "").strip()
        prefer_down_view = self._is_above_stage(stage)
        evidence_items = [
            self._format_detection_evidence("front", front_detection, front_image),
            self._format_detection_evidence("down", down_detection, down_image),
        ]
        if prefer_down_view:
            evidence_items = [evidence_items[1], evidence_items[0]]
        evidence = "\n".join(evidence_items)
        user_text = USER_PROMPT.format(
            mode=mode or "unknown",
            instruction=instruction,
            target=target,
            relation=relation or "none",
            completion_condition=completion_condition or "none",
            arrival_radius_m=f"{self.stop_depth:.2f}",
            evidence=evidence,
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
            if accepted_view not in {"front", "down", "none"}:
                accepted_view = "none"
            if not target_detected:
                accepted_view = "none"
            done = bool(data.get("done", False)) if target_detected else False
            reason = "complete" if done else "not complete"
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

        def append_front():
            content.append({"type": "text", "text": "Front-view image with proposed detection box overlay:"})
            content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{self._to_b64(self._overlay_detection(front_image, front_detection, 'front'))}"
                },
            })

        def append_down():
            if down_image is None:
                return
            content.append({"type": "text", "text": "Down-view image with proposed detection box overlay:"})
            content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{self._to_b64(self._overlay_detection(down_image, down_detection, 'down'))}"
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
        color = (255, 64, 64) if view_name == "front" else (64, 200, 255)
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
        return (
            f"- {view_name}: image_size={image_size}, visible={bool(detection.visible)}, "
            f"bbox_px={bbox}, score={float(detection.score or 0.0):.2f}, "
            f"depth={depth}, bbox_area_ratio={area_ratio}, bbox_center_norm={center}, label={label}"
        )

