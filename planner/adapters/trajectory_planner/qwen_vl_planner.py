from __future__ import annotations

import logging
import time

import requests

from planner.adapters.trajectory_planner.prompts import NavigationTrajectoryPrompt, TrajectoryPromptBuilder
from planner.adapters.trajectory_planner.response_parser import parse_trajectory
from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.execution import ExecutionBudget
from planner.domain.visual_goal import VisualGoalPlanningContext
from planner.errors import BudgetExceeded, ProtocolError
from planner.adapters.image_encoding import image_url

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

    def plan(
        self,
        observation: Observation,
        instruction: str,
        *,
        current_guidance: str | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
        visual_goal_context: VisualGoalPlanningContext | None = None,
        budget: ExecutionBudget | None = None,
    ) -> RelativeTrajectory:
        logger.info("[TrajectoryPlanner] model=%s instruction=%s", self.model, instruction)
        logger.info("[TrajectoryPlanner] RGB=%s depth=%s pose=%s", observation.rgb.size, observation.depth_image.size, observation.vehicle_pose)
        system_prompt = self.prompt_builder.system_prompt()
        if visual_goal_context is not None and getattr(self.prompt_builder, "supports_visual_goal_context", False) is not True:
            raise ProtocolError("backend prompt does not support visual-goal context")
        extra = {} if visual_goal_context is None else {"visual_goal_context": visual_goal_context}
        user_prompt = self.prompt_builder.build_user_prompt(
            instruction,
            observation,
            self.expected_points,
            current_guidance=current_guidance,
            round_index=round_index,
            initial_pose=initial_pose or observation.vehicle_pose,
            previous_trajectory=previous_trajectory,
            **extra,
        )
        content = [
            {"type": "image_url", "image_url": {"url": _image_url(observation.rgb, "JPEG")}},
            {"type": "image_url", "image_url": {"url": _image_url(observation.depth_image, "PNG")}},
            {"type": "text", "text": user_prompt},
        ]
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        started = time.perf_counter()
        timeout = self.timeout_s if budget is None else min(self.timeout_s, budget.remaining(time.monotonic()))
        if timeout <= 0:
            raise BudgetExceeded("deadline before trajectory request")
        response = requests.post(
            self.url,
            headers=headers,
            json={
                "model": self.model,
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": content},
                ],
                "temperature": 0.0,
                "max_tokens": 256,
                **({"thinking": {"type": self.thinking}} if self.thinking is not None else {}),
            },
            timeout=timeout,
        )
        response.raise_for_status()
        if budget is not None and budget.remaining(time.monotonic()) <= 0:
            raise BudgetExceeded("late trajectory response discarded")
        choice = response.json().get("choices", [{}])[0]
        raw = choice.get("message", {}).get("content", "")
        logger.info("[TrajectoryPlanner] %.2fs finish_reason=%s raw response:\n%s", time.perf_counter() - started, choice.get("finish_reason"), raw)
        return parse_trajectory(raw, self.expected_points)


'''
把一张 PIL 图片转换成 Base64 编码的 data:image/...;base64,... 字符串，
从而可以直接作为图像 URL 发送给 Qwen/OpenAI 兼容的多模态接口
'''
def _image_url(image, image_format: str) -> str:
    return image_url(image, image_format)

'''确保你传入的基础 API 地址最后一定指向 /chat/completions 接口'''
def _chat_url(url: str) -> str:
    value = str(url).rstrip("/")
    return value if value.endswith("/chat/completions") else f"{value}/chat/completions"
