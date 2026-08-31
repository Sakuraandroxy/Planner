"""Sliding-window Qwen planner and trajectory queue helpers.

The sliding-window LoRA uses incremental body-frame waypoints anchored at the
current front-view pose: the first waypoint is relative to the current UAV pose,
and each following waypoint is relative to the previous waypoint. The runtime
stores absolute world waypoints so Qwen results can be anchored with the pose
and yaw captured when the slow planning request was submitted.
"""

from __future__ import annotations

import ast
import base64
import io
import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence

import requests
from urllib.parse import urlparse

from agent.functions.common.config_access import first_value, function_section
from agent.functions.common.trajectory import normalize_xyz_waypoints
from agent.models.planner import register_planner
from agent.models.planner.base import BasePlanner, TrajectoryResult
from config import cfg


def incremental_to_cumulative(waypoints: Iterable[Sequence[float]] | None) -> List[List[float]]:
    """Convert adjacent incremental body-frame waypoints to cumulative offsets."""
    cumulative: List[List[float]] = []
    x = y = z = 0.0
    for wp in normalize_xyz_waypoints(waypoints):
        x += wp[0]
        y += wp[1]
        z += wp[2]
        cumulative.append([round(x, 3), round(y, 3), round(z, 3)])
    return cumulative


def cumulative_to_incremental(waypoints: Iterable[Sequence[float]] | None) -> List[List[float]]:
    """Convert cumulative body-frame waypoints to adjacent incremental offsets."""
    incremental: List[List[float]] = []
    prev = [0.0, 0.0, 0.0]
    for wp in normalize_xyz_waypoints(waypoints):
        delta = [
            round(wp[0] - prev[0], 3),
            round(wp[1] - prev[1], 3),
            round(wp[2] - prev[2], 3),
        ]
        if any(abs(v) >= 1e-6 for v in delta):
            incremental.append(delta)
        prev = wp
    return incremental


def _yaw_rotation_matrix(yaw_deg: float) -> List[List[float]]:
    yaw = math.radians(float(yaw_deg or 0.0))
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _rotation_matrix(rot_body_to_world, yaw_deg: float) -> List[List[float]]:
    return _yaw_rotation_matrix(yaw_deg)


def cumulative_body_to_world(
    waypoints: Iterable[Sequence[float]] | None,
    start_pos: Sequence[float],
    start_yaw_deg: float,
    start_rot_body_to_world=None,
) -> List[List[float]]:
    normalized = normalize_xyz_waypoints(waypoints)
    if not normalized:
        return []
    rot = _rotation_matrix(start_rot_body_to_world, start_yaw_deg)
    sx, sy, sz = float(start_pos[0]), float(start_pos[1]), float(start_pos[2])
    out: List[List[float]] = []
    for x, y, z in normalized:
        wx = sx + rot[0][0] * x + rot[0][1] * y + rot[0][2] * z
        wy = sy + rot[1][0] * x + rot[1][1] * y + rot[1][2] * z
        wz = sz + rot[2][0] * x + rot[2][1] * y + rot[2][2] * z
        out.append([
            round(wx, 3),
            round(wy, 3),
            round(wz, 3),
        ])
    return out


def world_to_cumulative_body(
    world_positions: Iterable[Sequence[float]] | None,
    current_pos: Sequence[float],
    current_yaw_deg: float,
    current_rot_body_to_world=None,
) -> List[List[float]]:
    """Convert absolute world positions into current-pose cumulative body offsets."""
    positions = list(world_positions or [])
    if not positions:
        return []
    rot = _rotation_matrix(current_rot_body_to_world, current_yaw_deg)
    sx, sy, sz = float(current_pos[0]), float(current_pos[1]), float(current_pos[2])
    out: List[List[float]] = []
    for wp in positions:
        if not isinstance(wp, (list, tuple)) or len(wp) < 3:
            continue
        dx = float(wp[0]) - sx
        dy = float(wp[1]) - sy
        dz = float(wp[2]) - sz
        bx = rot[0][0] * dx + rot[1][0] * dy + rot[2][0] * dz
        by = rot[0][1] * dx + rot[1][1] * dy + rot[2][1] * dz
        bz = rot[0][2] * dx + rot[1][2] * dy + rot[2][2] * dz
        out.append([round(bx, 3), round(by, 3), round(bz, 3)])
    return normalize_xyz_waypoints(out)


@dataclass
class SlidingWindowTrajectoryQueue:
    """World-coordinate queue used between sliding Qwen and AirSim execution."""

    max_pending: int = 5
    execute_count: int = 1
    world_waypoints: List[List[float]] = field(default_factory=list)

    def clear(self) -> None:
        self.world_waypoints.clear()

    def pending_incremental(
        self,
        current_pos: Sequence[float],
        current_yaw_deg: float,
        current_rot_body_to_world=None,
    ) -> List[List[float]]:
        pending_world = self.world_waypoints[: max(0, int(self.max_pending))]
        cumulative = world_to_cumulative_body(
            pending_world,
            current_pos,
            current_yaw_deg,
            current_rot_body_to_world=current_rot_body_to_world,
        )
        return cumulative_to_incremental(cumulative)

    def append_incremental_output(
        self,
        additional_incremental: Iterable[Sequence[float]] | None,
        anchor_pos: Sequence[float],
        anchor_yaw_deg: float,
        anchor_world_waypoints: Iterable[Sequence[float]] | None = None,
        anchor_rot_body_to_world=None,
    ) -> int:
        additional = normalize_xyz_waypoints(additional_incremental)
        if not additional:
            return 0

        # Use the frozen pending-world snapshot (anchor_world_waypoints)
        # as the basis for the body-frame prefix sum, NOT the current
        # self.world_waypoints which may have been partially consumed by
        # the fast executor.  This matches what Qwen saw as "pending" at
        # submission time.
        anchor_world = list(anchor_world_waypoints or self.world_waypoints)
        existing_cumulative = world_to_cumulative_body(
            anchor_world,
            anchor_pos,
            anchor_yaw_deg,
            current_rot_body_to_world=anchor_rot_body_to_world,
        )
        anchor = existing_cumulative[-1] if existing_cumulative else [0.0, 0.0, 0.0]
        x, y, z = float(anchor[0]), float(anchor[1]), float(anchor[2])
        new_cumulative: List[List[float]] = []
        for dx, dy, dz in additional:
            x += dx
            y += dy
            z += dz
            new_cumulative.append([round(x, 3), round(y, 3), round(z, 3)])

        new_world = cumulative_body_to_world(
            new_cumulative,
            anchor_pos,
            anchor_yaw_deg,
            start_rot_body_to_world=anchor_rot_body_to_world,
        )
        self.world_waypoints.extend(new_world)
        return len(new_world)

    def next_execution_cumulative(
        self,
        current_pos: Sequence[float],
        current_yaw_deg: float,
        count: int | None = None,
        current_rot_body_to_world=None,
    ) -> List[List[float]]:
        n = int(count if count is not None else self.execute_count)
        n = max(1, n)
        return world_to_cumulative_body(
            self.world_waypoints[:n],
            current_pos,
            current_yaw_deg,
            current_rot_body_to_world=current_rot_body_to_world,
        )

    def mark_executed(self, count: int | None = None) -> None:
        n = int(count if count is not None else self.execute_count)
        if n <= 0:
            return
        del self.world_waypoints[:n]

    def summary(self) -> str:
        return f"queue={len(self.world_waypoints)} pending_max={self.max_pending} exec={self.execute_count} format=incremental_body"


@register_planner("qwen_sliding_window_planner")
class SlidingWindowQwenPlanner(BasePlanner):
    """Qwen planner matching the sliding-window fine-tuning prompt."""

    def __init__(self):
        ag = cfg.get("AGENT", {})
        pc = function_section(cfg, "PLANNING")
        self.url = self._resolve_chat_url(pc, ag)
        self.timeout = int(first_value(pc.get("TIMEOUT"), ag.get("PLANNER_TIMEOUT"), default=120))
        self.model = str(first_value(pc.get("MODEL_NAME"), ag.get("PLANNER_MODEL"), default="qwen-vl"))
        self.api_key = str(first_value(pc.get("API_KEY"), ag.get("PLANNER_API_KEY"), default="no-key"))
        self.max_additional = 5

    @staticmethod
    def _chat_url(url: str) -> str:
        return str(url or "").strip()

    @classmethod
    def _resolve_chat_url(cls, pc: dict, ag: dict) -> str:
        explicit = str(first_value(pc.get("URL"), default="")).strip()
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

    def _prepare_image(self, img):
        if img.mode != "RGB":
            img = img.convert("RGB")
        return img

    def _image_url(self, img) -> str:
        img = self._prepare_image(img)
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
        geometry_context: dict | None = None,
    ) -> str:
        pending = normalize_xyz_waypoints(pending_waypoints)[:5]
        pending_str = "[" + ", ".join(
            "[" + ", ".join(f"{v:.2f}" for v in wp) + "]"
            for wp in pending
        ) + "]"
        parts = [
            f"Instruction: {(instruction or '').strip()}",
        ]
        direction = str(direction or "").strip()
        if direction:
            parts.append(f"Direction:{direction}")
        memory_hint = str(memory_hint or "").strip()
        if memory_hint:
            parts.append(memory_hint)
        if geometry_context:
            parts.append(
                "Unified camera/navigation geometry (JSON): "
                + json.dumps(geometry_context, ensure_ascii=False, separators=(",", ":"))
            )
        if pending:
            parts.append(f"Pending incremental body-frame waypoints: {pending_str}")
        parts.extend([
            "Output exactly 5 additional incremental body-frame waypoints as a JSON list.",
            (
                "Each waypoint must be [dx, dy, dz], where the first pending "
                "or output waypoint is relative to the current horizontal navigation frame "
                "and each following waypoint is relative to the previous waypoint."
            ),
            (
                "The navigation frame is independent of every camera pose: +dx is current heading, "
                "+dy is right, and AirSim NED +dz is down. Use camera optical axes and the supplied "
                "world/navigation geometry instead of assuming any image is horizontal or front-facing."
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

    def plan(self, front_img, down_img, instruction: str,
             direction: str = "", detected_bbox=None,
             depth_meters=None, detection=None,
             down_depth_meters=None,
             relation: str = "", target: str = "",
             pending_waypoints=None,
             memory_hint: str = "",
             camera_images=None,
             geometry_context: dict | None = None,
             print_prompt: bool = True) -> TrajectoryResult:
        prompt = self._build_prompt(
            instruction,
            pending_waypoints,
            direction=direction,
            memory_hint=memory_hint,
            geometry_context=geometry_context,
        )
        images = []
        for image in list(camera_images or [front_img, down_img if down_img is not None else front_img]):
            if image is not None and all(image is not existing for existing in images):
                images.append(image)
        if not images and front_img is not None:
            images = [front_img]
        if print_prompt:
            print("[QwenSlidingPrompt]")
            print("<image>" * len(images) + prompt)
        content = [
            {"type": "image_url", "image_url": {"url": self._image_url(image)}}
            for image in images
        ]
        content.append({"type": "text", "text": prompt})
        payload = {
            "model": self.model,
            "messages": [{"role": "user", "content": content}],
            "max_tokens": 256,
            "temperature": 0.0,
        }
        headers = {}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"

        t0 = time.time()
        try:
            resp = requests.post(self.url, json=payload, headers=headers, timeout=self.timeout)
            resp.raise_for_status()
            data = resp.json()
        except Exception as exc:
            elapsed = time.time() - t0
            print(f"  [QwenSliding] API error after {elapsed:.2f}s url={self.url!r}: {exc}")
            result = TrajectoryResult(
                waypoints=[],
                done=False,
                reasoning=f"QwenSliding api error: {exc}",
            )
            result.elapsed_s = elapsed
            result.pending_count = len(normalize_xyz_waypoints(pending_waypoints))
            return result
        elapsed = time.time() - t0
        raw_output = data.get("choices", [{}])[0].get("message", {}).get("content", "")
        waypoints = self._parse_waypoints(raw_output)[: self.max_additional]

        server_time = data.get("time_s", elapsed)
        if not waypoints:
            print(f"  [QwenSliding] raw_output: {raw_output[:300]}")

        result = TrajectoryResult(
            waypoints=waypoints,
            done=False,
            reasoning=f"QwenSliding: {len(waypoints)} incremental body waypoints",
        )
        result.elapsed_s = float(server_time)
        result.pending_count = len(normalize_xyz_waypoints(pending_waypoints))
        return result
