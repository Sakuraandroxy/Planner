from __future__ import annotations

import logging
import time

import requests

logger = logging.getLogger(__name__)

from planner.adapters.task_parser.prompt import user_prompt
from planner.adapters.task_parser.response_schema import parse_json_object
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.adapters.task_parser.tasks import TaskCodec, builtin_task_codecs
from planner.domain.mission import MissionPlan
from planner.domain.mission import TaskKind
from planner.adapters.task_parser.prompt import task_parser_system_prompt
from planner.errors import ProtocolError
from planner.adapters.chat_request_options import generation_options

#把用户的自然语言指令发给任务解析大模型
class OpenAITaskParser:
    def __init__(self, url: str, model: str, api_key: str, timeout_s: float, supported_kinds=None, *,
                 max_completion_tokens: int | None = None, reasoning_effort: str | None = None,
                 codecs: tuple[TaskCodec, ...] | None = None, request_format: str = "openai"):
        self.url = _chat_url(url)
        self.model = model
        self.api_key = api_key
        self.timeout_s = timeout_s
        self.max_completion_tokens = max_completion_tokens
        self.reasoning_effort = reasoning_effort
        generation_options(max_completion_tokens, reasoning_effort, request_format)
        self.request_format = request_format
        kinds = {TaskKind.BASIC_MOTION} if supported_kinds is None else set(supported_kinds)
        available = builtin_task_codecs() if codecs is None else codecs
        self.codecs = tuple(codec for codec in available if codec.kind in kinds)
        self.system_prompt = task_parser_system_prompt(kinds, codecs=self.codecs)

    def parse(self, instruction: str) -> MissionPlan:
        if not isinstance(instruction, str) or not instruction.strip():
            raise ProtocolError("instruction is empty")
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        logger.info("[TaskParser] model=%s instruction=%s", self.model, instruction)
        started = time.perf_counter()
        try:
            return self._request(instruction, headers, started)
        except requests.RequestException as exc:
            raise ProtocolError(f"task parser request failed ({type(exc).__name__})") from exc

    def _request(self, instruction: str, headers: dict, started: float) -> MissionPlan:
        options = generation_options(self.max_completion_tokens, self.reasoning_effort, self.request_format)
        response = requests.post(
            self.url,
            headers=headers,
            json={
                "model": self.model,
                "messages": [
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_prompt(instruction)},
                ],
                **options,
            },
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        try:
            choice = response.json()["choices"][0]
            message = choice["message"]
            if choice.get("finish_reason") not in (None, "stop") or message.get("refusal"):
                raise ProtocolError("task parser response was truncated or refused")
            content = message["content"]
            if not isinstance(content, str) or not content.strip():
                raise ProtocolError("task parser response has no mission text")
        except (ValueError, KeyError, IndexError, TypeError, AttributeError) as exc:
            raise ProtocolError("task parser response has no valid text choice") from exc
        logger.info("[TaskParser] %.2fs raw response:\n%s", time.perf_counter() - started, content)
        return mission_from_dict(parse_json_object(content), codecs=self.codecs)


def _chat_url(url: str) -> str:
    value = str(url).rstrip("/")
    return value if value.endswith("/chat/completions") else f"{value}/chat/completions"

