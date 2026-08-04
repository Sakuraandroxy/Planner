"""OpenAI-compatible sliding-window Qwen planner backend.

Despite the historical name, this planner now follows the same input/output
contract as ``SlidingWindowQwenPlanner``: two images plus the sliding-window
prompt in, incremental body-frame ``[dx, dy, dz]`` waypoints out.
"""

from __future__ import annotations

import ast
import base64
import io
import re
import time
from typing import Iterable, List, Sequence
from urllib.parse import urlparse

import requests

from agent.functions.common.config_access import first_value, function_section
from agent.functions.common.trajectory import normalize_xyz_waypoints
from agent.models.planner import register_planner
from agent.models.planner.base import BasePlanner, TrajectoryResult
from config import cfg


@register_planner("api_atomic_planner")
class ApiAtomicPlanner(BasePlanner):
    """Call an OpenAI-compatible chat API using the sliding-window Qwen prompt."""

    def __init__(self):
        ag = cfg.get("AGENT", {}) or {}
        pc = function_section(cfg, "PLANNING")
        self.url = self._resolve_chat_url(pc, ag)
        self.api_key = str(first_value(pc.get("API_KEY"), ag.get("PLANNER_API_KEY"), default="no-key"))
        self.model = str(first_value(pc.get("MODEL_NAME"), ag.get("PLANNER_MODEL"), default="qwen-vl"))
        self.max_tokens = int(first_value(pc.get("MAX_TOKENS"), ag.get("PLANNER_MAX_TOKENS"), default=256))
        self.timeout = int(first_value(pc.get("TIMEOUT"), ag.get("PLANNER_TIMEOUT"), default=120))

    @staticmethod
    def _chat_url(url: str) -> str:
        return str(url or "").strip()

    @classmethod
    def _resolve_chat_url(cls, pc: dict, ag: dict) -> str:
        explicit = str(first_value(pc.get("URL"), ag.get("PLANNER_URL"), default="")).strip()
        if explicit:
            return cls._chat_url(explicit)

        host = str(first_value(pc.get("SERVER_IP"), default="")).strip()
        port = str(first_value(pc.get("SERVER_PORT"), default="")).strip()
        path = str(first_value(pc.get("CHAT_PATH"), default="/v1/chat/completions")).strip() or "/v1/chat/completions"
        if host:
            if host.startswith("http://") or host.startswith("https://"):
                base = host.rstrip("/")
            else:
                base = f"http://{host}"
            parsed = urlparse(base)
            if port and not parsed.port:
                base = f"{base}:{port}"
            return f"{base}{path if path.startswith('/') else '/' + path}"

        return cls._chat_url(str(ag.get("PLANNER_URL", "")).strip())

    @staticmethod
    def _prepare_image(img):
        if img is None:
            return None
        if img.mode != "RGB":
            img = img.convert("RGB")
        return img

    def _image_url(self, img) -> str:
        img = self._prepare_image(img)
        if img is None:
            return ""
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=90)
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")
        return f"data:image/jpeg;base64,{b64}"

    def _build_prompt(
        self,
        instruction: str,
        pending_waypoints: Iterable[Sequence[float]] | None,
        direction: str = "",
        memory_hint: str = "",
    ) -> str:
        pending = normalize_xyz_waypoints(pending_waypoints)[:5]
        pending_str = "[" + ", ".join(
            "[" + ", ".join(f"{v:.2f}" for v in wp) + "]"
            for wp in pending
        ) + "]"
        parts = [f"Instruction: {(instruction or '').strip()}"]
        direction = str(direction or "").strip()
        if direction:
            parts.append(f"Direction:{direction}")
        memory_hint = str(memory_hint or "").strip()
        if memory_hint:
            parts.append(memory_hint)
        if pending:
            parts.append(f"Pending incremental body-frame waypoints: {pending_str}")
        parts.extend([
            "Output exactly 5 additional incremental body-frame waypoints as a JSON list.",
            (
                "Each waypoint must be [dx, dy, dz], where the first pending "
                "or output waypoint is relative to the current drone position/front-view frame "
                "and each following waypoint is relative to the previous waypoint."
            ),
            "Do not output any other text.",
        ])
        return "\n".join(parts)

    @staticmethod
    def _format_prompt_for_log(prompt: str) -> str:
        return f"<image><image>{prompt}"

    @staticmethod
    def _parse_waypoints(text: str) -> List[List[float]]:
        match = re.search(r"\[\s*\[.*?\]\s*\]", text or "", re.DOTALL)
        if not match:
            return []
        try:
            parsed = ast.literal_eval(match.group(0))
        except Exception:
            return []
        return normalize_xyz_waypoints(parsed)

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
        pending_waypoints=None,
        memory_hint: str = "",
        print_prompt: bool = True,
    ) -> TrajectoryResult:
        started = time.time()
        prompt = self._build_prompt(instruction, pending_waypoints, direction=direction, memory_hint=memory_hint)
        if print_prompt:
            print("[QwenSlidingPrompt]")
            print(self._format_prompt_for_log(prompt))

        front_url = self._image_url(front_img)
        down_url = self._image_url(down_img if down_img is not None else front_img)
        content = []
        if front_url:
            content.append({"type": "image_url", "image_url": {"url": front_url}})
        if down_url:
            content.append({"type": "image_url", "image_url": {"url": down_url}})
        content.append({"type": "text", "text": prompt})

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
            resp = requests.post(self.url, json=payload, headers=headers, timeout=self.timeout)
            resp.raise_for_status()
            data = resp.json()
            raw = data["choices"][0]["message"]["content"]
        except Exception as exc:
            print(f"  [ApiAtomicPlanner] API error: {exc}")
            return TrajectoryResult(waypoints=[], done=False, reasoning=f"api error: {exc}")

        waypoints = self._parse_waypoints(raw)
        elapsed = time.time() - started
        print(
            f"  [ApiAtomicPlanner] {elapsed:.2f}s -> wp={len(waypoints)} "
            f"raw={raw[:200].replace(chr(10), ' ')}"
        )
        return TrajectoryResult(
            waypoints=waypoints,
            done=False,
            reasoning=raw,
            actions=[],
            candidates=[],
        )


# Compatibility for registries that refer to the older all-caps spelling.
APIAtomicPlanner = ApiAtomicPlanner
