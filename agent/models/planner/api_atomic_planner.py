"""OpenAI-compatible atomic-action planner backend."""

from __future__ import annotations

import base64
import io
import json
import re
import time
from typing import List

from openai import OpenAI

from agent.functions.common.image_encoder import get_cached_down_b64, get_cached_front_b64
from agent.functions.common.config_access import first_value, function_section
from agent.functions.common.trajectory import actions_to_cumulative_body_waypoints, trajectory_delta
from agent.models.planner import register_planner
from agent.models.planner.base import BasePlanner, TrajectoryResult

PLANNER_SYSTEM_PROMPT = """You are a UAV path planner.
Return only valid JSON with:
{"selected_index": 0, "done": false, "candidates": [{"actions": ["forward 5"], "reason": "...", "scale": 1.0}]}
Allowed actions: forward X, backward X, left X, right X, up X, down X.
"""


@register_planner("api_atomic_planner")
class ApiAtomicPlanner(BasePlanner):
    """Plan atomic actions through an OpenAI-compatible VLM API."""

    def __init__(self):
        from config import cfg

        ag = cfg.get("AGENT", {}) or {}
        pc = function_section(cfg, "PLANNING")
        self.client = OpenAI(
            base_url=first_value(pc.get("URL"), ag.get("PLANNER_URL"), default=""),
            api_key=first_value(pc.get("API_KEY"), ag.get("PLANNER_API_KEY"), default="no-key"),
        )
        self.model = str(first_value(pc.get("MODEL_NAME"), ag.get("PLANNER_MODEL"), default=""))
        self.max_tokens = int(first_value(pc.get("MAX_TOKENS"), ag.get("PLANNER_MAX_TOKENS"), default=2048))
        self.stop_threshold = float(first_value(pc.get("STOP_DEPTH_THRESHOLD"), ag.get("STOP_DEPTH_THRESHOLD"), default=8.0))
        self.candidate_count = 1

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
        started = time.time()
        content = []
        for img, getter in ((front_img, get_cached_front_b64), (down_img, get_cached_down_b64)):
            b64 = _b64(img, getter)
            if b64:
                content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}})
        content.append({"type": "text", "text": f"Instruction: {instruction}\nDirection hint: {direction}\nReturn {self.candidate_count} candidates."})

        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "system", "content": PLANNER_SYSTEM_PROMPT}, {"role": "user", "content": content}],
                max_tokens=self.max_tokens,
                temperature=0.0,
                extra_body={"thinking": {"type": "disabled"}},
            )
            raw = resp.choices[0].message.content or ""
        except Exception as exc:
            print(f"  [ApiAtomicPlanner] API error: {exc}")
            return TrajectoryResult(waypoints=[], done=False, reasoning=f"api error: {exc}")

        data = _parse_json(raw)
        candidates = _parse_candidates(data.get("candidates", []))
        selected = int(data.get("selected_index", 0) or 0)
        selected = max(0, min(selected, len(candidates) - 1)) if candidates else 0
        chosen = candidates[selected] if candidates else {"actions": [], "waypoints": []}
        elapsed = time.time() - started
        print(f"  [ApiAtomicPlanner] {elapsed:.2f}s -> {len(candidates)} candidates, selected={selected}")
        return TrajectoryResult(
            waypoints=chosen.get("waypoints", []),
            done=bool(data.get("done", False)),
            reasoning=str(data.get("reasoning_summary", "")),
            actions=chosen.get("actions", []),
            candidates=candidates,
        )


def _b64(img, cache_getter=None):
    if img is None:
        return None
    if cache_getter:
        cached = cache_getter()
        if cached:
            return cached
    if img.mode == "RGBA":
        img = img.convert("RGB")
    elif img.mode != "RGB":
        img = img.convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def _parse_json(raw: str) -> dict:
    text = re.sub(r"```(?:json)?\s*", "", raw or "")
    text = re.sub(r"```\s*", "", text)
    match = re.search(r"\{.*\}", text, re.S)
    if not match:
        return {}
    try:
        return json.loads(match.group())
    except json.JSONDecodeError:
        return {}


def _parse_candidates(raw_candidates) -> List[dict]:
    out = []
    for raw in raw_candidates or []:
        if not isinstance(raw, dict):
            continue
        actions = [str(a) for a in raw.get("actions", [])]
        waypoints = actions_to_cumulative_body_waypoints(actions)
        out.append(
            {
                "actions": actions,
                "reason": str(raw.get("reason", "")),
                "scale": float(raw.get("scale", 1.0) or 1.0),
                "waypoints": waypoints,
                "delta": trajectory_delta(waypoints),
                "source": "api_atomic_planner",
            }
        )
    return out
