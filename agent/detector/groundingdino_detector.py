"""GroundingDINO 检测器——调用服务器 API，不依赖旧 groundindino_client.py。"""
import base64, io, os, time, re
from typing import List, Optional, Tuple
from PIL import Image
import requests

from agent.detector.base import BaseDetector, DetectionResult


# ─── 公共常量 ───
from config import cfg
GROUNDINGDINO_URL = cfg["AGENT"]["GROUNDINGDINO_URL"]
GROUNDINGDINO_TIMEOUT = int(cfg["AGENT"].get("DETECTOR_TIMEOUT", 15))
GROUNDINGDINO_BOX_THRESHOLD = float(cfg["AGENT"].get("DETECTOR_BOX_THRESHOLD", 0.4))
GROUNDINGDINO_TEXT_THRESHOLD = float(cfg["AGENT"].get("DETECTOR_TEXT_THRESHOLD", 0.3))

# 目标翻译由 VLM 任务解析器在输出 target 字段时完成（英文）



from agent.detector import register_detector


@register_detector("groundingdino")
class GroundingDINODetector(BaseDetector):
    """基于 GroundingDINO 服务器 HTTP API 的目标检测器。"""

    def detect(self, image: Image.Image, caption: str,
               depth_meters=None, camera_name: str = "front") -> DetectionResult:
        if image.mode == "RGBA":
            image = image.convert("RGB")
        effective = caption  # target 已由 VLM 任务解析器输出英文

        # 编码并调用 API（优先复用 ImageEncoder 缓存）
        from agent.common.image_encoder import get_cached_down_b64, get_cached_front_b64
        cached = get_cached_front_b64() if camera_name == "front" else get_cached_down_b64()
        if cached:
            b64 = cached
        else:
            buf = io.BytesIO()
            image.save(buf, format="JPEG", quality=90)
            b64 = base64.b64encode(buf.getvalue()).decode()

        resp = requests.post(GROUNDINGDINO_URL, json={
            "image": b64, "caption": effective,
            "box_threshold": GROUNDINGDINO_BOX_THRESHOLD,
            "text_threshold": GROUNDINGDINO_TEXT_THRESHOLD
        }, timeout=GROUNDINGDINO_TIMEOUT)
        data = resp.json()

        if not data.get("success") or not data.get("detections"):
            return DetectionResult(visible=False, camera=camera_name)

        best = data["detections"][0]
        bbox = best["bbox"]
        score = best["score"]
        label = best.get("phrase", effective)

        # 深度计算
        depth_median = None
        depth_bbox = None
        if depth_meters is not None:
            import numpy as np
            h, w = depth_meters.shape
            # 深度图的 bbox 按比例缩放
            sw = w / image.width
            sh = h / image.height
            db = [int(bbox[0]*sw), int(bbox[1]*sh), int(bbox[2]*sw), int(bbox[3]*sh)]
            depth_bbox = db
            cx, cy = (db[0] + db[2]) // 2, (db[1] + db[3]) // 2
            if 0 <= cy < h and 0 <= cx < w:
                depth_median = float(depth_meters[cy, cx])

        return DetectionResult(
            visible=True, bbox=bbox, score=score,
            label=label, depth_median=depth_median, depth_bbox=depth_bbox,
            camera=camera_name,
        )

    def detect_all(self, image: Image.Image, caption: str,
                   depth_meters=None, camera_name: str = "front") -> List[DetectionResult]:
        if image.mode == "RGBA":
            image = image.convert("RGB")
        effective = caption  # target 已由 VLM 任务解析器输出英文
        from agent.common.image_encoder import get_cached_down_b64, get_cached_front_b64
        b64 = get_cached_front_b64() if camera_name == "front" else get_cached_down_b64()
        if not b64:
            buf = io.BytesIO()
            image.save(buf, format="JPEG", quality=90)
            b64 = base64.b64encode(buf.getvalue()).decode()

        resp = requests.post(GROUNDINGDINO_URL, json={
            "image": b64, "caption": effective,
            "box_threshold": GROUNDINGDINO_BOX_THRESHOLD,
            "text_threshold": GROUNDINGDINO_TEXT_THRESHOLD
        }, timeout=GROUNDINGDINO_TIMEOUT)
        data = resp.json()

        results = []
        if not data.get("success") or not data.get("detections"):
            return results

        for d in data["detections"]:
            bbox = d["bbox"]
            # 深度计算 —— 与 detect() 保持一致
            depth_median = None
            depth_bbox = None
            if depth_meters is not None:
                import numpy as np
                h, w = depth_meters.shape
                sw = w / image.width
                sh = h / image.height
                db = [int(bbox[0]*sw), int(bbox[1]*sh), int(bbox[2]*sw), int(bbox[3]*sh)]
                depth_bbox = db
                cx, cy = (db[0] + db[2]) // 2, (db[1] + db[3]) // 2
                if 0 <= cy < h and 0 <= cx < w:
                    depth_median = float(depth_meters[cy, cx])

            results.append(DetectionResult(
                visible=True, bbox=bbox, score=d["score"],
                label=d.get("phrase", effective),
                depth_median=depth_median, depth_bbox=depth_bbox,
                camera=camera_name,
            ))
        return results
