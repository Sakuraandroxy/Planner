"""OpenAI-compatible visual completion checker with VLM bbox + program depth."""

from __future__ import annotations

import base64
import io
import json
import re
import time
from typing import Any, Dict, Optional

from openai import OpenAI

from agent.completion.task_completion import CompletionResult
from agent.detector.base import DetectionResult


SYSTEM_PROMPT = """You are a UAV target localization judge.

Your only job is to inspect the current front-view and down-view images and decide:
1. whether the requested target is actually visible now, and
2. in which view the best valid target box appears.

Do not decide trajectory. Do not decide distance threshold completion from geometry.
Be conservative. Do not blindly trust large boxes covering most of the image.
For above / over / on top stages, the best completion evidence should come from the down-view: the target's main body should be clearly visible near the central area, not just a small edge/corner/partial fragment at the image border.
If the target is visible, output the best valid bbox for each view where it is visible.
If only one view is reliable, set accepted_view to that view even if the other view has a larger or higher-confidence-looking region.
If neither view reliably contains the requested target, set target_detected=false and accepted_view="none".

Return exactly one JSON object with exactly these keys:
{"target_detected": true or false, "accepted_view": "front" or "down" or "none", "front_bbox_norm": [x1,y1,x2,y2] or null, "down_bbox_norm": [x1,y1,x2,y2] or null}
"""


USER_PROMPT = """Stage mode: {mode}
Stage instruction: {instruction}
Target: {target}
Relation: {relation}
Completion condition: {completion_condition}

Locate the requested target in the front-view and down-view images.
Return only {{"target_detected": true or false, "accepted_view": "front" or "down" or "none", "front_bbox_norm": [x1,y1,x2,y2] or null, "down_bbox_norm": [x1,y1,x2,y2] or null}}.
No reason. No explanation."""


class ApiTaskCompletionChecker:
    """Pure VLM bbox proposal + programmatic depth threshold completion."""

    name = "api_completion"
    uses_detector = False

    def __init__(self, cfg: dict):
        tc = cfg.get("TASK_COMPLETION", {})
        ag = cfg.get("AGENT", {})
        self.enabled = bool(tc.get("ENABLED", True))
        self.rgb_profile = str(tc.get("RGB_CAPTURE_PROFILE", tc.get("CAPTURE_PROFILE", "front_down")))
        self.depth_profile = str(tc.get("DEPTH_CAPTURE_PROFILE", "front_down_both_depth"))
        self.capture_profile = self.rgb_profile
        self.stop_depth = float(tc.get("STOP_DEPTH_THRESHOLD", ag.get("STOP_DEPTH_THRESHOLD", 8.0)))
        self.url = str(tc.get("URL", "") or "").strip()
        self.model = str(tc.get("MODEL", "") or "").strip()
        self.api_key = str(tc.get("API_KEY", "no-key") or "no-key")
        self.timeout = float(tc.get("TIMEOUT", 60))
        self.max_tokens = int(tc.get("MAX_TOKENS", 256))
        self.temperature = float(tc.get("TEMPERATURE", 0.0))

    def is_detector_enabled(self) -> bool:
        return False

    def should_check_stage(self, stage: Any) -> bool:
        return bool(self.enabled and stage and getattr(stage, "mode", "") in {"target", "detect"})

    def capture_profile_for_stage(self, stage: Any) -> str:
        if self.should_check_stage(stage) and getattr(stage, "mode", "") == "target":
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
        analysis = self.analyze_rgb(stage, fallback_instruction, front_image, down_image)
        result = self.finalize_analysis(
            stage,
            analysis,
            front_image,
            down_image,
            front_depth_meters=front_depth_meters,
            down_depth_meters=down_depth_meters,
        )
        result.elapsed = time.perf_counter() - started
        return result

    def evaluate_with_detection(
        self,
        stage: Any,
        fallback_instruction: str,
        front_image,
        down_image,
        detection=None,
        front_detection=None,
        down_detection=None,
        front_depth_meters=None,
        down_depth_meters=None,
    ) -> CompletionResult:
        return self.evaluate(
            stage,
            fallback_instruction,
            front_image,
            down_image,
            front_depth_meters=front_depth_meters,
            down_depth_meters=down_depth_meters,
        )

    def analyze_rgb(self, stage: Any, fallback_instruction: str, front_image, down_image) -> Dict[str, Any]:
        if not self.should_check_stage(stage):
            return {"error": "completion disabled or non-target stage"}
        if front_image is None:
            return {"error": "front image unavailable"}
        if not self.url or not self.model:
            return {"error": "api completion URL/model not configured"}

        instruction = (getattr(stage, "instruction", "") or fallback_instruction or "").strip()
        target = (getattr(stage, "target_query", "") or getattr(stage, "target", "") or "").strip()
        relation = (getattr(stage, "relation", "") or "").strip()
        completion_condition = (getattr(stage, "completion_condition", "") or "").strip()
        mode = (getattr(stage, "mode", "") or "").strip()

        user_text = USER_PROMPT.format(
            mode=mode or "unknown",
            instruction=instruction,
            target=target,
            relation=relation or "none",
            completion_condition=completion_condition or "none",
        )
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": self._build_user_content(user_text, front_image, down_image)},
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
            data["ok"] = True
            return data
        except Exception as exc:
            return {"error": f"api completion failed: {exc}", "ok": False}

    def finalize_analysis(
        self,
        stage: Any,
        analysis: Dict[str, Any],
        front_image,
        down_image,
        front_depth_meters=None,
        down_depth_meters=None,
    ) -> CompletionResult:
        if not self.should_check_stage(stage):
            return CompletionResult(reason="completion disabled or non-target stage")
        if not analysis or analysis.get("ok") is False:
            return CompletionResult(
                checked=True,
                done=False,
                target_detected=None,
                accepted_view="none",
                reason=str((analysis or {}).get("error", "api completion failed")),
            )

        front_detection = self._detection_from_norm(
            analysis.get("front_bbox_norm"),
            front_image,
            "front",
            front_depth_meters,
        )
        down_detection = self._detection_from_norm(
            analysis.get("down_bbox_norm"),
            down_image,
            "down",
            down_depth_meters,
        )
        target_detected = bool(analysis.get("target_detected", False))
        accepted_view = str(analysis.get("accepted_view", "none") or "none").strip().lower()
        if accepted_view not in {"front", "down", "none"}:
            accepted_view = "none"
        if not target_detected:
            accepted_view = "none"

        detection = None
        if accepted_view == "front":
            detection = front_detection
        elif accepted_view == "down":
            detection = down_detection

        mode = getattr(stage, "mode", "") or ""
        if mode == "detect":
            done = bool(target_detected and detection and detection.visible)
            return CompletionResult(
                checked=True,
                done=done,
                target_detected=target_detected,
                accepted_view=accepted_view,
                reason="complete" if done else "not complete",
                detection=detection,
                direction="" if accepted_view != "down" else "Target is visible in the downward view below the drone.",
            )

        if self._is_above_stage(stage):
            done = bool(
                target_detected
                and accepted_view == "down"
                and detection is not None
                and detection.visible
                and detection.depth_median is not None
                and detection.depth_median < self.stop_depth
            )
        else:
            done = bool(
                target_detected
                and detection is not None
                and detection.visible
                and detection.depth_median is not None
                and detection.depth_median < self.stop_depth
            )
        return CompletionResult(
            checked=True,
            done=done,
            target_detected=target_detected,
            accepted_view=accepted_view,
            reason="complete" if done else "not complete",
            detection=detection,
            direction="" if accepted_view != "down" else "Target is visible in the downward view below the drone.",
        )

    def _build_user_content(self, text: str, front_image, down_image):
        content = [{"type": "text", "text": text}]
        content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/jpeg;base64,{self._to_b64(front_image)}"},
        })
        if down_image is not None:
            content.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{self._to_b64(down_image)}"},
            })
        return content

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
                data["target_detected"] = False
            if "accepted_view" not in data:
                data["accepted_view"] = "none"
            data.setdefault("front_bbox_norm", None)
            data.setdefault("down_bbox_norm", None)
            return data
        return {
            "target_detected": False,
            "accepted_view": "none",
            "front_bbox_norm": None,
            "down_bbox_norm": None,
        }

    @staticmethod
    def _detection_from_norm(bbox_norm, image, camera_name: str, depth_meters=None) -> Optional[DetectionResult]:
        if image is None or not bbox_norm or len(bbox_norm) != 4:
            return None
        try:
            vals = [float(v) for v in bbox_norm]
        except Exception:
            return None
        vals = [max(0.0, min(1.0, v)) for v in vals]
        x1 = int(round(vals[0] * image.width))
        y1 = int(round(vals[1] * image.height))
        x2 = int(round(vals[2] * image.width))
        y2 = int(round(vals[3] * image.height))
        x1 = max(0, min(x1, image.width - 1))
        y1 = max(0, min(y1, image.height - 1))
        x2 = max(x1 + 1, min(x2, image.width))
        y2 = max(y1 + 1, min(y2, image.height))
        depth_median = None
        depth_bbox = None
        if depth_meters is not None:
            try:
                h, w = depth_meters.shape
                sw = w / float(image.width)
                sh = h / float(image.height)
                db = [int(x1 * sw), int(y1 * sh), int(x2 * sw), int(y2 * sh)]
                depth_bbox = db
                cx = (db[0] + db[2]) // 2
                cy = (db[1] + db[3]) // 2
                if 0 <= cy < h and 0 <= cx < w:
                    depth_median = float(depth_meters[cy, cx])
            except Exception:
                depth_median = None
                depth_bbox = None
        return DetectionResult(
            visible=True,
            bbox=[x1, y1, x2, y2],
            score=1.0,
            label="vlm_target",
            depth_median=depth_median,
            depth_bbox=depth_bbox,
            camera=camera_name,
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
