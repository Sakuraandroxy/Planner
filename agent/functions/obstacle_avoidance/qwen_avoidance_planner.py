"""Qwen local obstacle-avoidance planner and API adapter.

This backend proposes a replacement for an unsafe sliding-window suffix.  It
never writes to the execution queue and it does not claim that a proposal is
safe; the runtime safety shield validates the returned body-frame path before
committing it.
"""

from __future__ import annotations

import base64
import io
import json
import math
import re
import time
from typing import Any, Iterable, Sequence
from urllib.parse import urlparse

import requests

from agent.functions.common.config_access import as_bool, first_value, function_section
from agent.functions.common.trajectory import normalize_xyz_waypoints
from agent.functions.obstacle_avoidance.schemas import AvoidanceContext, AvoidancePlan
from config import cfg


class QwenAvoidancePlanner:
    """Call an OpenAI-compatible Qwen-VL endpoint for a local detour."""

    def __init__(self, config: dict | None = None):
        self.config = dict(config or {})
        ag = cfg.get("AGENT", {}) or {}
        planning = function_section(cfg, "PLANNING")
        self.url = self._resolve_url(self.config, planning, ag)
        self.api_key = str(first_value(
            self.config.get("QWEN_AVOIDANCE_API_KEY"),
            self.config.get("API_KEY"),
            planning.get("API_KEY"),
            ag.get("PLANNER_API_KEY"),
            default="no-key",
        ))
        self.model = str(first_value(
            self.config.get("QWEN_AVOIDANCE_MODEL_NAME"),
            self.config.get("MODEL_NAME"),
            planning.get("MODEL_NAME"),
            ag.get("PLANNER_MODEL"),
            default="qwen-vl",
        ))
        self.timeout_s = max(1.0, float(first_value(
            self.config.get("QWEN_AVOIDANCE_TIMEOUT_S"),
            self.config.get("TIMEOUT_S"),
            self.config.get("TIMEOUT"),
            default=8.0,
        )))
        self.max_waypoints = max(1, int(self.config.get("MAX_WAYPOINTS", 5)))
        self.max_tokens = max(64, int(self.config.get("MAX_TOKENS", 256)))
        self.depth_image_enabled = as_bool(
            self.config.get("DEPTH_IMAGE_ENABLED", True),
            True,
        )

    @classmethod
    def _resolve_url(cls, local: dict, planning: dict, ag: dict) -> str:
        explicit = str(first_value(
            local.get("QWEN_AVOIDANCE_URL"),
            local.get("URL"),
            local.get("CHAT_URL"),
            planning.get("URL"),
            ag.get("PLANNER_URL"),
            default="",
        )).strip()
        if explicit:
            return explicit
        host = str(first_value(
            local.get("QWEN_AVOIDANCE_SERVER_IP"),
            local.get("SERVER_IP"),
            planning.get("SERVER_IP"),
            default="",
        )).strip()
        port = str(first_value(
            local.get("QWEN_AVOIDANCE_SERVER_PORT"),
            local.get("SERVER_PORT"),
            planning.get("SERVER_PORT"),
            default="",
        )).strip()
        path = str(first_value(
            local.get("QWEN_AVOIDANCE_CHAT_PATH"),
            local.get("CHAT_PATH"),
            planning.get("CHAT_PATH"),
            default="/v1/chat/completions",
        )).strip() or "/v1/chat/completions"
        if not host:
            return ""
        base = host.rstrip("/") if host.startswith(("http://", "https://")) else f"http://{host}"
        if port and not urlparse(base).port:
            base = f"{base}:{port}"
        return f"{base}{path if path.startswith('/') else '/' + path}"

    @staticmethod
    def _image(value):
        if value is None:
            return None
        try:
            from PIL import Image

            if hasattr(value, "convert"):
                return value.convert("RGB")
            if isinstance(value, (bytes, bytearray)):
                with Image.open(io.BytesIO(bytes(value))) as image:
                    return image.convert("RGB").copy()
        except Exception:
            return None
        return None

    @classmethod
    def _image_url(cls, value) -> str:
        image = cls._image(value)
        if image is None:
            return ""
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=88)
        encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
        return f"data:image/jpeg;base64,{encoded}"

    @classmethod
    def _depth_overlay(cls, depth_meters, *, max_depth_m: float = 28.0):
        """Create a compact depth visualization without retaining raw depth."""
        if depth_meters is None:
            return None
        try:
            import numpy as np
            from PIL import Image

            depth = np.asarray(depth_meters, dtype=np.float32)
            if depth.ndim != 2 or depth.size == 0:
                return None
            valid = np.isfinite(depth) & (depth > 0.1) & (depth <= max_depth_m)
            normalized = np.ones(depth.shape, dtype=np.float32)
            normalized[valid] = np.clip(depth[valid] / max(max_depth_m, 1.0), 0.0, 1.0)
            # Near obstacles are bright; invalid/far pixels stay dark.
            gray = ((1.0 - normalized) * 255.0).astype(np.uint8)
            gray[~valid] = 0
            green = np.minimum(255, gray.astype(np.uint16) * 2).astype(np.uint8)
            rgb = np.stack([gray, green, 255 - gray], axis=-1)
            return Image.fromarray(rgb.astype(np.uint8), mode="RGB")
        except Exception:
            return None

    def _build_prompt(
        self,
        instruction: str,
        reference_path_body: Iterable[Sequence[float]] | None,
        context: AvoidanceContext | dict | None,
        *,
        target: str = "",
        memory_hint: str = "",
        attempt: int = 0,
    ) -> str:
        path = normalize_xyz_waypoints(reference_path_body)[: max(1, int(self.config.get("REFERENCE_WAYPOINTS", 8)))]
        path_text = "[" + ", ".join(
            "[" + ", ".join(f"{float(v):.2f}" for v in point[:3]) + "]"
            for point in path
        ) + "]"
        context_dict = context.to_dict() if isinstance(context, AvoidanceContext) else dict(context or {})
        return "\n".join([
            "You are the local collision-avoidance planner for a UAV.",
            "Safety has priority over the mission instruction and the old route.",
            "The depth overlay and numeric obstacle summary are authoritative; do not infer a free corridor from RGB alone.",
            f"Mission instruction: {(instruction or '').strip()}",
            f"Target: {(target or '').strip()}",
            f"Memory hint: {(memory_hint or '').strip()}",
            f"Reference cumulative body-frame route (may be blocked): {path_text}",
            f"Obstacle summary: {json.dumps(context_dict, ensure_ascii=False, separators=(',', ':'))}",
            f"Avoidance attempt: {int(attempt)}",
            "Return JSON only with this schema:",
            '{"action":"detour|hold|scan","waypoints":[[x,y,z]],"rejoin":true,"rejoin_index":null,"confidence":0.0}',
            (
                "waypoints are cumulative body-frame positions from the current request pose; "
                "use short segments, keep every segment outside the observed obstacle volume, "
                "and never continue through the blocked reference segment."
            ),
            "If no corridor is visibly and metrically safe, return action=hold.",
        ])

    @staticmethod
    def _content_text(content: Any) -> str:
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            return "".join(
                str(item.get("text", "")) if isinstance(item, dict) else str(item)
                for item in content
            )
        return str(content or "")

    @classmethod
    def _parse_payload(cls, payload: Any) -> dict:
        if isinstance(payload, dict) and isinstance(payload.get("waypoints"), list):
            return payload
        if isinstance(payload, list):
            return {"action": "detour", "waypoints": payload}
        text = cls._content_text(payload)
        fenced = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL | re.IGNORECASE)
        candidates = [fenced.group(1)] if fenced else []
        candidates.append(text.strip())
        for candidate in candidates:
            try:
                parsed = json.loads(candidate)
                if isinstance(parsed, dict):
                    return parsed
            except Exception:
                continue
        # A compatibility fallback for endpoints that return a bare array.
        match = re.search(r"\[\s*\[.*?\]\s*\]", text, re.DOTALL)
        if match:
            try:
                parsed = json.loads(match.group(0))
                if isinstance(parsed, list):
                    return {"action": "detour", "waypoints": parsed}
            except Exception:
                pass
        return {}

    def _normalize_plan(self, payload: Any, *, elapsed_s: float, raw: Any = None) -> AvoidancePlan:
        data = self._parse_payload(payload)
        action = str(data.get("action", "detour" if data.get("waypoints") else "hold")).strip().lower()
        if action not in {"detour", "hold", "scan"}:
            action = "hold"
        coordinate_limit = max(1.0, float(self.config.get("MAX_COORDINATE_M", 40.0)))
        waypoints = [
            point for point in normalize_xyz_waypoints(data.get("waypoints", []))
            if all(math.isfinite(float(value)) and abs(float(value)) <= coordinate_limit for value in point[:3])
        ][: self.max_waypoints]
        max_segment_m = max(0.5, float(self.config.get("MAX_SEGMENT_M", 10.0)))
        previous = [0.0, 0.0, 0.0]
        if any(
            math.sqrt(sum((float(point[i]) - float(previous_point[i])) ** 2 for i in range(3))) > max_segment_m
            for previous_point, point in zip([previous] + waypoints[:-1], waypoints)
        ):
            waypoints = []
        confidence = data.get("confidence", 0.0)
        try:
            confidence = max(0.0, min(1.0, float(confidence)))
        except Exception:
            confidence = 0.0
        rejoin_index = data.get("rejoin_index")
        try:
            rejoin_index = None if rejoin_index is None else int(rejoin_index)
        except Exception:
            rejoin_index = None
        if action == "detour" and not waypoints:
            action = "hold"
        if action != "detour":
            # A model occasionally emits action=hold together with speculative
            # waypoints.  Hold is authoritative: no movement may leak through
            # that contradictory payload.
            waypoints = []
        return AvoidancePlan(
            action=action,
            waypoints=waypoints,
            rejoin=bool(data.get("rejoin", True)),
            rejoin_index=rejoin_index,
            confidence=confidence,
            reasoning=str(data.get("reason", "") or "")[:500],
            raw=raw,
            error="" if data else "unparseable avoidance response",
            elapsed_s=max(0.0, float(elapsed_s)),
        )

    def plan(
        self,
        *,
        front_image=None,
        down_image=None,
        depth_meters=None,
        instruction: str = "",
        reference_path_body: Iterable[Sequence[float]] | None = None,
        context: AvoidanceContext | dict | None = None,
        target: str = "",
        memory_hint: str = "",
        attempt: int = 0,
    ) -> AvoidancePlan:
        if not self.url:
            return AvoidancePlan(action="hold", error="avoidance Qwen URL is empty")

        started = time.perf_counter()
        front_url = self._image_url(front_image)
        down_url = self._image_url(down_image)
        depth_overlay = self._depth_overlay(
            depth_meters,
            max_depth_m=float(self.config.get("DEPTH_OVERLAY_MAX_M", 28.0)),
        ) if self.depth_image_enabled else None
        depth_url = self._image_url(depth_overlay)
        content = []
        for url in (front_url, depth_url, down_url):
            if url:
                content.append({"type": "image_url", "image_url": {"url": url}})
        content.append({
            "type": "text",
            "text": self._build_prompt(
                instruction,
                reference_path_body,
                context,
                target=target,
                memory_hint=memory_hint,
                attempt=attempt,
            ),
        })
        payload = {
            "model": self.model,
            "messages": [{"role": "user", "content": content}],
            "max_tokens": self.max_tokens,
            "temperature": 0.0,
        }
        headers = {}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        try:
            response = requests.post(
                self.url,
                json=payload,
                headers=headers,
                timeout=self.timeout_s,
            )
            response.raise_for_status()
            data = response.json()
            if isinstance(data, dict) and "choices" in data:
                content = data.get("choices", [{}])[0].get("message", {}).get("content", "")
            else:
                content = data
            result = self._normalize_plan(
                content,
                elapsed_s=time.perf_counter() - started,
                raw=data,
            )
            print(
                f"  [QwenAvoidance] {time.perf_counter() - started:.2f}s "
                f"action={result.action} wp={len(result.waypoints)}"
            )
            return result
        except Exception as exc:
            elapsed = time.perf_counter() - started
            print(f"  [QwenAvoidance] API error after {elapsed:.2f}s: {exc}")
            return AvoidancePlan(
                action="hold",
                error=str(exc),
                reasoning="avoidance request failed",
            )


__all__ = ["QwenAvoidancePlanner"]
