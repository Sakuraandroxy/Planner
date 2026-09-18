from __future__ import annotations

import base64
import io

import requests

from planner.adapters.trajectory_planner.prompt import build_prompt
from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.domain.observation import Observation
from planner.domain.trajectory import RelativeTrajectory


class QwenVLPlanner:
    def __init__(self, url: str, model: str, api_key: str, timeout_s: float, expected_points: int = 5):
        self.url = _chat_url(url)
        self.model = model
        self.api_key = api_key
        self.timeout_s = timeout_s
        self.expected_points = expected_points

    def plan(self, observation: Observation, instruction: str) -> RelativeTrajectory:
        content = [
            {"type": "image_url", "image_url": {"url": _image_url(observation.rgb, "JPEG")}},
            {"type": "image_url", "image_url": {"url": _image_url(observation.depth_image, "PNG")}},
            {"type": "text", "text": build_prompt(instruction, observation)},
        ]
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        response = requests.post(
            self.url,
            headers=headers,
            json={
                "model": self.model,
                "messages": [{"role": "user", "content": content}],
                "temperature": 0.0,
                "max_tokens": 256,
            },
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        raw = response.json().get("choices", [{}])[0].get("message", {}).get("content", "")
        return parse_trajectory(raw, self.expected_points)


def _image_url(image, image_format: str) -> str:
    converted = image.convert("RGB") if image.mode != "RGB" else image
    buffer = io.BytesIO()
    save_options = {"quality": 90} if image_format == "JPEG" else {}
    converted.save(buffer, format=image_format, **save_options)
    mime = "jpeg" if image_format == "JPEG" else "png"
    return f"data:image/{mime};base64,{base64.b64encode(buffer.getvalue()).decode('ascii')}"


def _chat_url(url: str) -> str:
    value = str(url).rstrip("/")
    return value if value.endswith("/chat/completions") else f"{value}/chat/completions"
