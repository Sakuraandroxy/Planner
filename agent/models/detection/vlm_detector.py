"""VLM detector backend that asks a VLM to output one target bbox."""

from __future__ import annotations

import base64
import io
import json
import re
from typing import Optional

from PIL import Image
from openai import OpenAI

from agent.functions.common.image_encoder import get_cached_down_b64, get_cached_front_b64
from agent.functions.common.config_access import first_value, function_section
from agent.models.detection import register_detector
from agent.models.detection.base import BaseDetector, DetectionResult

TARGET_BBOX_SYSTEM_PROMPT = (
    "You are a target detector. Return only one JSON object. "
    "No markdown, no reasoning, no action planning."
)

TARGET_BBOX_PROMPT = """Locate the navigation target in this RGB image.
Target: {task_description}
Image size: width={image_width}, height={image_height}.

Return exactly:
{{"visible": true/false, "bbox_norm": [left, top, right, bottom] or null, "confidence": 0.0-1.0, "target": "name"}}

Rules:
- bbox_norm must be normalized to 0.0-1.0.
- If multiple same-class objects exist, choose the one described by the task.
- If the target is not visible, return visible=false, bbox_norm=null, confidence=0.0.
"""


@register_detector("vlm_detector")
class VLMDetector(BaseDetector):
    """Call an OpenAI-compatible VLM endpoint for target detection."""

    def __init__(self):
        from config import cfg

        fc = function_section(cfg, "PERCEPTION")
        ag = cfg.get("AGENT", {}) or {}
        self.client = OpenAI(
            base_url=first_value(fc.get("URL"), ag.get("DETECTOR_URL"), ag.get("PLANNER_URL"), default=""),
            api_key=first_value(fc.get("API_KEY"), ag.get("DETECTOR_API_KEY"), ag.get("PLANNER_API_KEY"), default="no-key"),
        )
        self.model = str(first_value(fc.get("MODEL_NAME"), ag.get("DETECTOR_MODEL"), ag.get("PLANNER_MODEL"), default=""))
        self.max_tokens = int(first_value(fc.get("MAX_TOKENS"), ag.get("DETECTOR_MAX_TOKENS"), default=512))

    def detect(self, image: Image.Image, caption: str, depth_meters=None, camera_name: str = "front") -> DetectionResult:
        if image.mode == "RGBA":
            image = image.convert("RGB")
        elif image.mode != "RGB":
            image = image.convert("RGB")
        w, h = image.size

        b64 = get_cached_front_b64() if camera_name == "front" else get_cached_down_b64()
        if not b64:
            buf = io.BytesIO()
            image.save(buf, format="JPEG", quality=85)
            b64 = base64.b64encode(buf.getvalue()).decode()

        messages = [
            {"role": "system", "content": TARGET_BBOX_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                    {
                        "type": "text",
                        "text": TARGET_BBOX_PROMPT.format(
                            task_description=caption,
                            image_width=w,
                            image_height=h,
                        ),
                    },
                ],
            },
        ]

        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=0.0,
                extra_body={"thinking": {"type": "disabled"}},
            )
            raw = resp.choices[0].message.content or ""
        except Exception as exc:
            print(f"  [VLMDetector] API error: {exc}")
            return DetectionResult(visible=False, camera=camera_name)

        result = self._parse_response(raw, image, depth_meters, caption)
        result.camera = camera_name
        return result

    def _parse_response(self, raw: str, image: Image.Image, depth_meters, caption: str = "") -> DetectionResult:
        text = re.sub(r"```(?:json)?\s*", "", raw or "")
        text = re.sub(r"```\s*", "", text)
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if not match:
            print(f"  [VLMDetector] no JSON in response: {(raw or '')[:200]}")
            return DetectionResult(visible=False, camera="none")

        try:
            data = json.loads(match.group(), strict=False)
        except json.JSONDecodeError:
            data = _relaxed_json_parse(match.group())
            if data is None:
                print(f"  [VLMDetector] parse failed: {match.group()[:200]}")
                return DetectionResult(visible=False, camera="none")

        if not bool(data.get("visible", False)):
            return DetectionResult(visible=False, camera="none")

        bbox_norm = data.get("bbox_norm") or data.get("bbox")
        if not bbox_norm or len(bbox_norm) != 4:
            return DetectionResult(visible=False, camera="none")

        bbox = _to_pixel_bbox(bbox_norm, image.size)
        if bbox is None:
            return DetectionResult(visible=False, camera="none")

        depth_median, depth_bbox = _depth_for_bbox(image, bbox, depth_meters)
        score = float(data.get("confidence", 0.0) or 0.0)
        label = str(data.get("target", caption) or caption)
        print(
            f"  [VLMDetector] target={label} bbox={bbox} score={score:.2f}"
            + (f" depth={depth_median:.1f}m" if depth_median is not None else "")
        )
        return DetectionResult(
            visible=True,
            bbox=bbox,
            score=score,
            label=label,
            depth_median=depth_median,
            depth_bbox=depth_bbox,
            camera="front",
        )


def _to_pixel_bbox(values, image_size):
    w, h = image_size
    try:
        vals = [float(v) for v in values]
    except (TypeError, ValueError):
        return None
    if all(0.0 <= v <= 1.0 for v in vals):
        x1, y1, x2, y2 = vals
        bbox = [int(round(x1 * w)), int(round(y1 * h)), int(round(x2 * w)), int(round(y2 * h))]
    else:
        bbox = [int(round(v)) for v in vals]
    bbox[0] = max(0, min(bbox[0], w - 1))
    bbox[1] = max(0, min(bbox[1], h - 1))
    bbox[2] = max(bbox[0] + 1, min(bbox[2], w))
    bbox[3] = max(bbox[1] + 1, min(bbox[3], h))
    return bbox


def _depth_for_bbox(image: Image.Image, bbox, depth_meters):
    if depth_meters is None:
        return None, None
    dh, dw = depth_meters.shape
    sw = dw / image.width
    sh = dh / image.height
    db = [int(bbox[0] * sw), int(bbox[1] * sh), int(bbox[2] * sw), int(bbox[3] * sh)]
    cx = (db[0] + db[2]) // 2
    cy = (db[1] + db[3]) // 2
    if 0 <= cy < dh and 0 <= cx < dw:
        return float(depth_meters[cy, cx]), db
    return None, db


def _relaxed_json_parse(raw: str) -> Optional[dict]:
    text = raw.strip()
    text = text.replace("None", "null").replace("True", "true").replace("False", "false")
    text = text.replace("'", '"')
    text = re.sub(r",\s*([}\]])", r"\1", text)
    text = re.sub(r"([{,])\s*([A-Za-z_][\w]*)\s*:", r'\1"\2":', text)
    try:
        return json.loads(text, strict=False)
    except json.JSONDecodeError:
        return None
