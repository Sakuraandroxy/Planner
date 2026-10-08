from __future__ import annotations

import logging
import time

import requests

logger = logging.getLogger(__name__)

from planner.adapters.task_parser.prompt import user_prompt
from planner.adapters.task_parser.response_schema import parse_json_object
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.domain.mission import MissionPlan
from planner.domain.mission import TaskKind
from planner.adapters.task_parser.prompt import task_parser_system_prompt
from planner.errors import ProtocolError

#把用户的自然语言指令发给任务解析大模型
class OpenAITaskParser:
    def __init__(self, url: str, model: str, api_key: str, timeout_s: float, supported_kinds=None):
        self.url = _chat_url(url)
        self.model = model
        self.api_key = api_key
        self.timeout_s = timeout_s
        self.system_prompt = task_parser_system_prompt(supported_kinds or {TaskKind.NAVIGATION})

    def parse(self, instruction: str) -> MissionPlan:
        if not instruction.strip():
            raise ProtocolError("instruction is empty")
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        logger.info("[TaskParser] model=%s instruction=%s", self.model, instruction)
        started = time.perf_counter()
        response = requests.post(
            self.url,
            headers=headers,
            json={
                "model": self.model,
                "messages": [
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_prompt(instruction)},
                ],
                "temperature": 0.0,
                "max_tokens": 1024,
            },
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        content = response.json().get("choices", [{}])[0].get("message", {}).get("content", "")
        logger.info("[TaskParser] %.2fs raw response:\n%s", time.perf_counter() - started, content)
        return mission_from_dict(parse_json_object(content))


def _chat_url(url: str) -> str:
    value = str(url).rstrip("/")
    return value if value.endswith("/chat/completions") else f"{value}/chat/completions"

