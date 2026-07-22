"""Qwen2.5-VL waypoint planner backend."""

from __future__ import annotations

import base64
import io
import time

from PIL import Image
import requests

from agent.functions.common.config_access import as_bool, first_value, function_section
from agent.models.planner import register_planner
from agent.models.planner.base import BasePlanner, TrajectoryResult
from config import cfg

_AG = cfg.get("AGENT", {}) or {}
_PLANNING = function_section(cfg, "PLANNING")
QWEN_URL = str(first_value(_PLANNING.get("URL"), _AG.get("PLANNER_URL"), default=""))
QWEN_WAYPOINT_COUNT = int(first_value(_PLANNING.get("WAYPOINT_COUNT"), _AG.get("QWEN_WAYPOINT_COUNT"), default=5))
QWEN_TIMEOUT = int(first_value(_PLANNING.get("TIMEOUT"), _AG.get("PLANNER_TIMEOUT"), default=120))
QWEN_RESIZE_ENABLED = as_bool(first_value(_PLANNING.get("RESIZE_ENABLED"), _AG.get("QWEN_RESIZE_ENABLED"), default=True), True)
QWEN_IMAGE_SIZE = int(first_value(_PLANNING.get("IMAGE_SIZE"), _AG.get("QWEN_IMAGE_SIZE"), default=1024))
QWEN_RESIZE_MODE = str(first_value(_PLANNING.get("RESIZE_MODE"), _AG.get("QWEN_RESIZE_MODE"), default="square")).strip().lower()


@register_planner("qwen_planner")
class QwenPlanner(BasePlanner):
    """Call the configured Qwen waypoint server."""

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
            return img.resize((max(1, int(round(w * scale))), max(1, int(round(h * scale)))), resample)
        return img.resize((QWEN_IMAGE_SIZE, QWEN_IMAGE_SIZE), resample)

    def plan(
        self,
        front_img,
        down_img,
        instruction: str,
        direction: str = "",
        detected_bbox=None,
        depth_meters=None,
        detection=None,
        down_depth_meters=None,
        relation: str = "",
        target: str = "",
    ) -> TrajectoryResult:
        def to_b64(img):
            img = self._prepare_image(img)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=90)
            return base64.b64encode(buf.getvalue()).decode()

        started = time.time()
        resp = requests.post(
            QWEN_URL,
            json={
                "front_image": to_b64(front_img),
                "down_image": to_b64(down_img) if down_img else to_b64(front_img),
                "instruction": (instruction or "").strip(),
                "waypoint_count": QWEN_WAYPOINT_COUNT,
            },
            timeout=QWEN_TIMEOUT,
        )
        data = resp.json()
        waypoints = list(data.get("waypoints", []) or [])[:QWEN_WAYPOINT_COUNT]
        while len(waypoints) < QWEN_WAYPOINT_COUNT:
            waypoints.append([0.0, 0.0, 0.0])

        raw_count = len(data.get("waypoints", []) or [])
        print(f"  [Qwen] {data.get('time_s', time.time() - started):.2f}s -> {QWEN_WAYPOINT_COUNT} waypoints ({raw_count} raw)")
        if raw_count == 0:
            print(f"  [Qwen] raw_output: {str(data.get('raw_output', ''))[:300]}")
        if raw_count == 0 or all(all(float(v) == 0.0 for v in wp) for wp in waypoints):
            print("  [Qwen] empty trajectory")

        done = self.should_stop(detected_bbox, depth_meters)
        return TrajectoryResult(
            waypoints=waypoints,
            done=done,
            reasoning=f"Qwen: {QWEN_WAYPOINT_COUNT} waypoints" + (" [arrived]" if done else ""),
        )
