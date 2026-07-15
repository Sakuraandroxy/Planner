"""Qwen2.5-VL 轨迹规划器——直接调用服务器 API，不依赖旧 qwen_client.py。"""
import base64, io, os, time
from typing import List, Optional
from PIL import Image
import requests

from agent.planner.base import BasePlanner, TrajectoryResult


from config import cfg
QWEN_URL = cfg["AGENT"]["PLANNER_URL"]
QWEN_WAYPOINT_COUNT = int(cfg["AGENT"]["QWEN_WAYPOINT_COUNT"])
QWEN_TIMEOUT = int(cfg["AGENT"].get("PLANNER_TIMEOUT", 120))
QWEN_RESIZE_ENABLED = bool(cfg["AGENT"].get("QWEN_RESIZE_ENABLED", True))
QWEN_IMAGE_SIZE = int(cfg["AGENT"].get("QWEN_IMAGE_SIZE", 1024))
QWEN_RESIZE_MODE = str(cfg["AGENT"].get("QWEN_RESIZE_MODE", "square")).strip().lower()


from agent.planner import register_planner


@register_planner("qwen_planner")
class QwenPlanner(BasePlanner):
    """基于 Qwen2.5-VL 的轨迹规划器。"""

    @staticmethod
    def _prepare_image(img):
        if img.mode == "RGBA":
            img = img.convert("RGB")
        elif img.mode != "RGB":
            img = img.convert("RGB")

        if not QWEN_RESIZE_ENABLED or QWEN_IMAGE_SIZE <= 0:
            return img

        resample = getattr(getattr(Image, "Resampling", Image), "BILINEAR")
        if QWEN_RESIZE_MODE == "long_edge":
            w, h = img.size
            long_edge = max(w, h)
            if long_edge <= 0 or long_edge == QWEN_IMAGE_SIZE:
                return img
            scale = QWEN_IMAGE_SIZE / float(long_edge)
            new_size = (max(1, int(round(w * scale))), max(1, int(round(h * scale))))
            return img.resize(new_size, resample)

        return img.resize((QWEN_IMAGE_SIZE, QWEN_IMAGE_SIZE), resample)

    def plan(self, front_img, down_img, instruction: str,
             direction: str = "", detected_bbox=None,
             depth_meters=None, detection=None,
             down_depth_meters=None,
             relation: str = "", target: str = "") -> TrajectoryResult:

        # 编码图像
        def to_b64(img):
            img = self._prepare_image(img)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=90)
            return base64.b64encode(buf.getvalue()).decode()

        # 编码图像
        def to_b64(img):
            img = self._prepare_image(img)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=90)
            return base64.b64encode(buf.getvalue()).decode()

        t0 = time.time()
        resp = requests.post(QWEN_URL, json={
            "front_image": to_b64(front_img),
            "down_image": to_b64(down_img) if down_img else to_b64(front_img),
            "instruction": (instruction or "").strip(),
            "waypoint_count": QWEN_WAYPOINT_COUNT,
        }, timeout=QWEN_TIMEOUT)
        data = resp.json()
        elapsed = time.time() - t0

        waypoints = data.get("waypoints", [])
        K = QWEN_WAYPOINT_COUNT
        waypoints = waypoints[:K]
        while len(waypoints) < K:
            waypoints.append([0.0, 0.0, 0.0])

        raw_count = len(data.get("waypoints", []))
        raw_out = data.get("raw_output", "")
        print(f"  [Qwen] {data.get('time_s', 0):.2f}s -> {K} waypoints ({raw_count} raw)")
        if raw_count == 0:
            print(f"  [Qwen] raw_output: {raw_out[:300]}")
        if raw_count == 0 or all(all(v == 0.0 for v in wp) for wp in waypoints):
            print(f"  [Qwen] 💤 空轨迹（LoRA 未生效）")

        done = self.should_stop(detected_bbox, depth_meters)
        return TrajectoryResult(
            waypoints=waypoints,
            done=done,
            reasoning=f"Qwen: {K} waypoints" + (" [到达]" if done else ""),
        )
