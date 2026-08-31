"""GroundingDINO HTTP detector backend."""

from __future__ import annotations

import base64
import io
from typing import List

from PIL import Image
import requests

from agent.functions.common.config_access import first_value, function_section
from agent.models.detection import register_detector
from agent.models.detection.base import BaseDetector, DetectionResult
from agent.functions.perception.camera_geometry import attach_detection_camera_context
from config import cfg


@register_detector("groundingdino")
class GroundingDINODetector(BaseDetector):
    """Call the configured GroundingDINO service and return DetectionResult."""

    def __init__(self):
        fc = function_section(cfg, "PERCEPTION")
        ag = cfg.get("AGENT", {}) or {}
        self.url = str(first_value(fc.get("URL"), ag.get("GROUNDINGDINO_URL"), default="")).strip()
        self.timeout = float(first_value(fc.get("TIMEOUT"), ag.get("DETECTOR_TIMEOUT"), default=15))
        self.connect_timeout = float(
            first_value(
                fc.get("CONNECT_TIMEOUT"),
                ag.get("DETECTOR_CONNECT_TIMEOUT"),
                default=min(5.0, self.timeout),
            )
        )
        self.box_threshold = float(first_value(fc.get("BOX_THRESHOLD"), ag.get("DETECTOR_BOX_THRESHOLD"), default=0.4))
        self.text_threshold = float(first_value(fc.get("TEXT_THRESHOLD"), ag.get("DETECTOR_TEXT_THRESHOLD"), default=0.3))

    def detect(self, image: Image.Image, caption: str, depth_meters=None, camera_name: str = "front") -> DetectionResult:
        results = self.detect_all(image, caption, depth_meters=depth_meters, camera_name=camera_name)
        return results[0] if results else DetectionResult(visible=False, camera=camera_name)

    def detect_all(
        self,
        image: Image.Image,
        caption: str,
        depth_meters=None,
        camera_name: str = "front",
    ) -> List[DetectionResult]:
        if image.mode == "RGBA":
            image = image.convert("RGB")
        elif image.mode != "RGB":
            image = image.convert("RGB")

        # Detection may run after planning has populated the global image
        # cache with an older frame. Always encode the image passed to this
        # call so a fresh completion/relocalization capture cannot silently be
        # replaced by stale Qwen input.
        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=90)
        b64 = base64.b64encode(buf.getvalue()).decode()

        resp = requests.post(
            self.url,
            json={
                "image": b64,
                "caption": caption,
                "box_threshold": self.box_threshold,
                "text_threshold": self.text_threshold,
            },
            # A missing route/service must be reported quickly so navigation
            # can keep its locked memory and enter a bounded retry backoff.
            # Model inference may still use the longer read timeout.
            timeout=(self.connect_timeout, self.timeout),
        )
        data = resp.json()
        detections = data.get("detections") if data.get("success") else None
        if not detections:
            return []

        results: List[DetectionResult] = []
        for item in detections:
            bbox = [int(round(v)) for v in item.get("bbox", [])[:4]]
            if len(bbox) != 4:
                continue
            depth_median, depth_bbox = _depth_for_bbox(image, bbox, depth_meters)
            results.append(
                attach_detection_camera_context(DetectionResult(
                    visible=True,
                    bbox=bbox,
                    score=float(item.get("score", 0.0) or 0.0),
                    label=str(item.get("phrase", caption) or caption),
                    depth_median=depth_median,
                    depth_bbox=depth_bbox,
                    camera=camera_name,
                ), image)
            )
        return results


def _depth_for_bbox(image: Image.Image, bbox, depth_meters):
    if depth_meters is None:
        return None, None
    h, w = depth_meters.shape
    sw = w / image.width
    sh = h / image.height
    db = [int(bbox[0] * sw), int(bbox[1] * sh), int(bbox[2] * sw), int(bbox[3] * sh)]
    cx = (db[0] + db[2]) // 2
    cy = (db[1] + db[3]) // 2
    if 0 <= cy < h and 0 <= cx < w:
        return float(depth_meters[cy, cx]), db
    return None, db
