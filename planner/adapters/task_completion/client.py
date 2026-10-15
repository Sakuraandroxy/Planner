"""OpenAI-compatible completion transport with an exact boolean wire protocol."""
from typing import Generic, TypeVar

from planner.adapters.multimodal_chat import MultimodalChatClient
from planner.errors import ProtocolError
from .prompt import CompletionPromptBuilder

RequestT = TypeVar("RequestT")


class OpenAICompatibleTaskCompletionChecker(Generic[RequestT]):
    def __init__(self, url, model, api_key, timeout_s, *,
                 prompt_builder: CompletionPromptBuilder[RequestT],
                 max_completion_tokens=2048, reasoning_effort="none", request_format="openai"):
        self.prompt_builder = prompt_builder
        self.transport = MultimodalChatClient(url, model, api_key, timeout_s,
            max_completion_tokens=max_completion_tokens, reasoning_effort=reasoning_effort,
            request_format=request_format)

    def check(self, request: RequestT, *, timeout_s: float) -> bool:
        messages = self.prompt_builder.prepare(request)
        raw = self.transport.request(messages.system, messages.user, messages.images,
            timeout_s=timeout_s, operation="task completion").strip()
        if raw not in ("true", "false"):
            raise ProtocolError("task completion must return exactly true or false")
        return raw == "true"
