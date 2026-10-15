"""Shared stateless HTTP transport for visual grounding and boolean completion."""
import math

import requests

from planner.adapters.image_encoding import image_url
from planner.errors import ProtocolError
from planner.adapters.chat_request_options import generation_options


class MultimodalChatClient:
    def __init__(self, url, model, api_key, timeout_s, *, max_completion_tokens=2048, reasoning_effort="low", request_format="openai"):
        value = url.rstrip("/")
        self.url = value if value.endswith("/chat/completions") else value + "/chat/completions"
        self.model, self.api_key, self.timeout_s = model, api_key, timeout_s
        self.max_completion_tokens, self.reasoning_effort = max_completion_tokens, reasoning_effort
        generation_options(max_completion_tokens, reasoning_effort, request_format)
        self.request_format = request_format

    def request(self, system: str, text: str, images: tuple, *, timeout_s: float, operation: str,
                max_completion_tokens: int | None = None, reasoning_effort: str | None = None) -> str:
        timeout = min(timeout_s, self.timeout_s)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ProtocolError(f"{operation} timeout must be finite and positive")
        content = [{"type": "image_url", "image_url": {"url": image_url(im, fmt)}} for im, fmt in images]
        content.append({"type": "text", "text": text})
        body = {"model": self.model,
                "messages": [{"role": "system", "content": system}, {"role": "user", "content": content}],
                **generation_options(
                    self.max_completion_tokens if max_completion_tokens is None else max_completion_tokens,
                    self.reasoning_effort if reasoning_effort is None else reasoning_effort, self.request_format)}
        try:
            response = requests.post(self.url, headers={"Authorization": f"Bearer {self.api_key}"},
                                     json=body, timeout=timeout)
            response.raise_for_status()
        except requests.RequestException as exc:
            raise ProtocolError(f"{operation} request failed ({type(exc).__name__})") from exc
        try:
            data = response.json()
            choice = data["choices"][0]
            if choice.get("finish_reason") not in (None, "stop") or choice["message"].get("refusal"):
                raise ProtocolError(f"{operation} response was truncated or refused")
            content = choice["message"]["content"]
            if not isinstance(content, str) or not content.strip():
                raise ProtocolError(f"{operation} response has no text")
            return content
        except (ValueError, KeyError, IndexError, TypeError, AttributeError) as exc:
            raise ProtocolError(f"{operation} response has no valid text choice") from exc
