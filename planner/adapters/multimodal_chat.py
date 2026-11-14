"""Shared stateless HTTP transport for visual grounding and boolean completion.

Supports the OpenAI-compatible ``/chat/completions`` wire format (also used by
the ``deepseek`` dialect) and the Anthropic Messages API.
"""
import math
import logging

import httpx

from planner.adapters.image_encoding import encoded_image_payload, image_url
from planner.errors import ProtocolError
from planner.adapters.chat_request_options import generation_options

ANTHROPIC_VERSION = "2023-06-01"


def chat_completions_url(url: str) -> str:
    value = str(url).rstrip("/")
    if not value:
        raise ValueError("model endpoint URL is empty")
    return value if value.endswith("/chat/completions") else value + "/chat/completions"


def anthropic_messages_url(url: str) -> str:
    value = str(url).rstrip("/")
    if not value:
        raise ValueError("model endpoint URL is empty")
    if value.endswith("/messages"):
        return value
    return value + ("/messages" if value.endswith("/v1") else "/v1/messages")


def anthropic_headers(api_key: str) -> dict:
    headers = {
        "content-type": "application/json",
        "anthropic-version": ANTHROPIC_VERSION,
    }
    if api_key and api_key != "no-key":
        headers["x-api-key"] = api_key
    return headers


class MultimodalChatClient:
    def __init__(self, url, model, api_key, timeout_s, *, max_completion_tokens=2048, reasoning_effort="none", request_format="openai"):
        if request_format == "anthropic":
            self.url = anthropic_messages_url(url)
        else:
            self.url = chat_completions_url(url)
        self.model, self.api_key, self.timeout_s = model, api_key, timeout_s
        self.max_completion_tokens, self.reasoning_effort = max_completion_tokens, reasoning_effort
        generation_options(max_completion_tokens, reasoning_effort, request_format)
        self.request_format = request_format

    def _body(self, system: str, text: str, images: tuple,
              max_completion_tokens, reasoning_effort) -> tuple[dict, dict]:
        options = generation_options(
            self.max_completion_tokens if max_completion_tokens is None else max_completion_tokens,
            self.reasoning_effort if reasoning_effort is None else reasoning_effort,
            self.request_format)
        if self.request_format == "anthropic":
            content = [
                {"type": "image", "source": {
                    "type": "base64",
                    "media_type": f"image/{mime}",
                    "data": data,
                }}
                for mime, data in (
                    encoded_image_payload(image, fmt=fmt) for image, fmt in images
                )
            ]
            content.append({"type": "text", "text": text})
            return {
                "model": self.model,
                "system": system,
                "messages": [{"role": "user", "content": content}],
                **options,
            }, anthropic_headers(self.api_key)
        content = [{"type": "image_url", "image_url": {"url": image_url(im, fmt)}} for im, fmt in images]
        content.append({"type": "text", "text": text})
        return {
            "model": self.model,
            "messages": [{"role": "system", "content": system}, {"role": "user", "content": content}],
            **options,
        }, (
            # h11 rejects "Bearer " (empty key with trailing space); the
            # reference only expects the header when a key is configured.
            {"Authorization": f"Bearer {self.api_key}"}
            if self.api_key
            else {}
        )

    def request(self, system: str, text: str, images: tuple, *, timeout_s: float, operation: str,
                max_completion_tokens: int | None = None, reasoning_effort: str | None = None) -> str:
        timeout = min(timeout_s, self.timeout_s)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ProtocolError(f"{operation} timeout must be finite and positive")
        body, headers = self._body(
            system, text, images, max_completion_tokens, reasoning_effort
        )
        try:
            response = httpx.post(self.url, headers=headers,
                                  json=body, timeout=timeout)
            response.raise_for_status()
        except httpx.HTTPError as exc:
            raise ProtocolError(f"{operation} request failed ({type(exc).__name__})") from exc
        try:
            data = response.json()
            usage = data.get("usage", {})
            if self.request_format == "anthropic":
                logging.getLogger(__name__).info(
                    "[%s] input_tokens=%s output_tokens=%s cache_read_tokens=%s",
                    operation, usage.get("input_tokens"),
                    usage.get("output_tokens"),
                    usage.get("cache_read_input_tokens"))
                return anthropic_response_text(data, operation)
            logging.getLogger(__name__).info("[%s] input_tokens=%s output_tokens=%s cache_hit_tokens=%s",
                operation, usage.get("prompt_tokens"), usage.get("completion_tokens"), usage.get("prompt_cache_hit_tokens"))
            choice = data["choices"][0]
            if choice.get("finish_reason") not in (None, "stop") or choice["message"].get("refusal"):
                raise ProtocolError(f"{operation} response was truncated or refused")
            content = choice["message"]["content"]
            if not isinstance(content, str) or not content.strip():
                raise ProtocolError(f"{operation} response has no text")
            return content
        except (ValueError, KeyError, IndexError, TypeError, AttributeError) as exc:
            raise ProtocolError(f"{operation} response has no valid text choice") from exc


def anthropic_response_text(data: dict, operation: str) -> str:
    blocks = data.get("content")
    if not isinstance(blocks, list):
        raise ProtocolError(f"{operation} response has no content blocks")
    if data.get("stop_reason") not in (None, "end_turn", "stop_sequence"):
        raise ProtocolError(f"{operation} response was truncated or refused")
    text = "".join(
        block.get("text", "")
        for block in blocks
        if isinstance(block, dict) and block.get("type") == "text"
    )
    if not text.strip():
        raise ProtocolError(f"{operation} response has no text")
    return text