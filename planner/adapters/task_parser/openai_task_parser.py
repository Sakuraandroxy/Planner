from __future__ import annotations

import requests

from planner.adapters.task_parser.prompt import SYSTEM_PROMPT, user_prompt
from planner.adapters.task_parser.response_schema import parse_json_object
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.domain.mission import MissionPlan
from planner.errors import ProtocolError


class OpenAITaskParser:
    def __init__(self, url: str, model: str, api_key: str, timeout_s: float):
        self.url = _chat_url(url)
        self.model = model
        self.api_key = api_key
        self.timeout_s = timeout_s

    def parse(self, instruction: str) -> MissionPlan:
        if not instruction.strip():
            raise ProtocolError("instruction is empty")
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        response = requests.post(
            self.url,
            headers=headers,
            json={
                "model": self.model,
                "messages": [
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt(instruction)},
                ],
                "temperature": 0.0,
                "max_tokens": 1024,
            },
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        content = response.json().get("choices", [{}])[0].get("message", {}).get("content", "")
        return mission_from_dict(parse_json_object(content))


def _chat_url(url: str) -> str:
    value = str(url).rstrip("/")
    return value if value.endswith("/chat/completions") else f"{value}/chat/completions"

