from __future__ import annotations

import base64
import io
import logging
import time

import requests

from planner.adapters.trajectory_planner.prompts import NavigationTrajectoryPrompt, TrajectoryPromptBuilder
from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.domain.observation import Observation
from planner.domain.trajectory import RelativeTrajectory

logger = logging.getLogger(__name__)


class QwenVLPlanner:
    def __init__(self, url: str, model: str, api_key: str, timeout_s: float,
                 expected_points: int = 5, thinking: str | None = None,
                 prompt_builder: TrajectoryPromptBuilder | None = None):
        self.url = _chat_url(url)
        self.model = model
        self.api_key = api_key
        self.timeout_s = timeout_s
        self.expected_points = expected_points
        self.thinking = thinking
        self.prompt_builder = prompt_builder or NavigationTrajectoryPrompt()

    def plan(self, observation: Observation, instruction: str) -> RelativeTrajectory:
        logger.info("[TrajectoryPlanner] model=%s instruction=%s", self.model, instruction)
        logger.info("[TrajectoryPlanner] RGB=%s depth=%s pose=%s", observation.rgb.size, observation.depth_image.size, observation.vehicle_pose)
        content = [
            {"type": "image_url", "image_url": {"url": _image_url(observation.rgb, "JPEG")}},
            {"type": "image_url", "image_url": {"url": _image_url(observation.depth_image, "PNG")}},
            {"type": "text", "text": self.prompt_builder.build(instruction, observation, self.expected_points)},
        ]
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        started = time.perf_counter()
        response = requests.post(
            self.url,
            headers=headers,
            json={
                "model": self.model,
                "messages": [{"role": "user", "content": content}],
                "temperature": 0.0,
                "max_tokens": 256,
                **({"thinking": {"type": self.thinking}} if self.thinking is not None else {}),
            },
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        choice = response.json().get("choices", [{}])[0]
        raw = choice.get("message", {}).get("content", "")
        logger.info("[TrajectoryPlanner] %.2fs finish_reason=%s raw response:\n%s", time.perf_counter() - started, choice.get("finish_reason"), raw)
        return parse_trajectory(raw, self.expected_points)


'''
把一张 PIL 图片转换成 Base64 编码的 data:image/...;base64,... 字符串，
从而可以直接作为图像 URL 发送给 Qwen/OpenAI 兼容的多模态接口
'''
def _image_url(image, image_format: str) -> str:
    converted = image.convert("RGB") if image.mode != "RGB" else image
    buffer = io.BytesIO()
    save_options = {"quality": 90} if image_format == "JPEG" else {}
    converted.save(buffer, format=image_format, **save_options)
    mime = "jpeg" if image_format == "JPEG" else "png"
    return f"data:image/{mime};base64,{base64.b64encode(buffer.getvalue()).decode('ascii')}"

'''确保你传入的基础 API 地址最后一定指向 /chat/completions 接口'''
def _chat_url(url: str) -> str:
    value = str(url).rstrip("/")
    return value if value.endswith("/chat/completions") else f"{value}/chat/completions"
