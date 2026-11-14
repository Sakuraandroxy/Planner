"""Minimal OpenAI-compatible chat transport shared by model adapters.

The transport only moves messages; prompt construction and response parsing
stay in the adapter that owns the protocol. API keys are read from the
environment so no credential ever lives in the repository.
"""

from __future__ import annotations

import math
import os
from typing import Protocol

import httpx

from ..errors import ProtocolError


class ChatTransport(Protocol):
    def complete(
        self,
        messages: list[dict],
        *,
        timeout_s: float,
        temperature: float = 0.0,
        max_tokens: int = 1024,
    ) -> str: ...


def chat_completions_url(url: str) -> str:
    value = str(url).rstrip("/")
    if not value:
        raise ValueError("model endpoint URL is empty")
    return (
        value
        if value.endswith("/chat/completions")
        else f"{value}/chat/completions"
    )


class HTTPChatTransport:
    """POST an OpenAI-compatible ``/chat/completions`` request."""

    def __init__(
        self,
        url: str,
        model: str,
        *,
        api_key: str | None = None,
        api_key_env: str | None = None,
        timeout_s: float = 30.0,
        client: httpx.Client | None = None,
    ):
        if not str(model).strip():
            raise ValueError("model name is empty")
        self.url = chat_completions_url(url)
        self.model = str(model)
        self.api_key = api_key or (
            os.environ.get(api_key_env, "") if api_key_env else ""
        )
        self.timeout_s = float(timeout_s)
        if not (math.isfinite(self.timeout_s) and self.timeout_s > 0):
            raise ValueError("transport timeout must be finite and positive")
        self._client = client

    def complete(
        self,
        messages: list[dict],
        *,
        timeout_s: float,
        temperature: float = 0.0,
        max_tokens: int = 1024,
    ) -> str:
        timeout = min(float(timeout_s), self.timeout_s)
        if not (math.isfinite(timeout) and timeout > 0):
            raise ProtocolError("model timeout must be finite and positive")
        headers = {"Content-Type": "application/json"}
        if self.api_key and self.api_key != "no-key":
            headers["Authorization"] = f"Bearer {self.api_key}"
        body = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": int(max_tokens),
        }
        try:
            if self._client is not None:
                response = self._client.post(
                    self.url, headers=headers, json=body, timeout=timeout
                )
            else:
                response = httpx.post(
                    self.url, headers=headers, json=body, timeout=timeout
                )
            response.raise_for_status()
        except httpx.HTTPError as error:
            raise ProtocolError(
                f"model request failed ({type(error).__name__})"
            ) from error
        try:
            data = response.json()
            choice = data["choices"][0]
        except (ValueError, KeyError, IndexError, TypeError) as error:
            raise ProtocolError("model response has no valid choice") from error
        if choice.get("finish_reason") not in (None, "stop"):
            raise ProtocolError("model response was truncated or filtered")
        message = choice.get("message") or {}
        if message.get("refusal"):
            raise ProtocolError("model refused the request")
        content = message.get("content")
        if not isinstance(content, str) or not content.strip():
            raise ProtocolError("model response has no text")
        return content