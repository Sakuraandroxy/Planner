"""Qwen2.5-VL 轨迹规划器——直接调用服务器 API，不依赖旧 qwen_client.py。"""
import base64, io, os, time
from typing import List, Optional
from PIL import Image
import requests

from agent.planner.base import BasePlanner, TrajectoryResult


from config import cfg
QWEN_URL = cfg["AGENT"]["PLANNER_URL"]
QWEN_WAYPOINT_COUNT = int(cfg["AGENT"]["QWEN_WAYPOINT_COUNT"])


from agent.planner import register_planner


@register_planner("qwen_planner")
class QwenPlanner(BasePlanner):
    """基于 Qwen2.5-VL 的轨迹规划器。"""

    def plan(self, front_img, down_img, instruction: str,
             direction: str = "", detected_bbox=None,
             depth_meters=None) -> TrajectoryResult:

        # 编码图像
        def to_b64(img):
            if img.mode == "RGBA":
                img = img.convert("RGB")
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=90)
            return base64.b64encode(buf.getvalue()).decode()

        t0 = time.time()
        resp = requests.post(QWEN_URL, json={
            "front_image": to_b64(front_img),
            "down_image": to_b64(down_img) if down_img else to_b64(front_img),
            "instruction": instruction,
            "direction": direction,
        }, timeout=120)
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
            print(f"  [Qwen] \U0001f4a4 空轨迹（LoRA 未生效）")

        done = self.should_stop(detected_bbox, depth_meters)
        return TrajectoryResult(
            waypoints=waypoints,
            done=done,
            reasoning=f"Qwen: {K} waypoints" + (" [\u5230\u8fbe]" if done else ""),
        )
