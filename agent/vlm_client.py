"""VLMClient - thin wrapper around the OpenAI chat completions API."""
import json
import time
from openai import OpenAI


class VLMClient:
    """Handles the HTTP/API layer for VLM calls."""

    def __init__(self, config):
        self.client = OpenAI(
            api_key=config.api_key or "no-key",
            base_url=config.base_url
        )
        self.model_name = config.model_name
        self.temperature = config.temperature
        self.thinking_mode = str(getattr(config, "thinking_mode", "disabled") or "default").lower()
        self.reasoning_effort = str(getattr(config, "reasoning_effort", "default") or "default").lower()
        self.enable_thinking = str(getattr(config, "enable_thinking", "default") or "default").lower()
        self.last_call_info = {}

    @staticmethod
    def _parse_bool(value):
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def describe_api_mode(self):
        """Return a short description of the current API request mode."""
        thinking_desc = self.thinking_mode if self.thinking_mode != "default" else "not_set"
        params = "model,messages,max_completion_tokens,temperature"
        if self.thinking_mode != "default":
            params += ",extra_body.thinking"
        if self.reasoning_effort != "default":
            params += ",extra_body.reasoning_effort"
        if self.enable_thinking != "default":
            params += ",extra_body.enable_thinking"
        return (
            f"model={self.model_name}, temperature={self.temperature}, "
            f"explicit_thinking_param={thinking_desc}, "
            f"reasoning_effort={self.reasoning_effort if self.reasoning_effort != 'default' else 'not_set'}, "
            f"enable_thinking={self.enable_thinking if self.enable_thinking != 'default' else 'not_set'}, "
            f"request_params=[{params}]"
        )

    def call(self, messages, max_tokens=8192):
        """Send messages to VLM, return (response_text, reasoning_content)."""
        started = time.perf_counter()
        response = None
        request_kwargs = {
            "model": self.model_name,
            "messages": messages,
            "max_completion_tokens": max_tokens,
            "temperature": self.temperature,
        }
        extra_body = {}
        if self.thinking_mode != "default":
            extra_body["thinking"] = {"type": self.thinking_mode}
        if self.reasoning_effort != "default":
            extra_body["reasoning_effort"] = self.reasoning_effort
        if self.enable_thinking != "default":
            extra_body["enable_thinking"] = self._parse_bool(self.enable_thinking)
        if extra_body:
            request_kwargs["extra_body"] = extra_body
        response = self.client.chat.completions.create(**request_kwargs)
        elapsed = time.perf_counter() - started
        choice = response.choices[0]
        response_text = choice.message.content or ""

        reasoning_content = ""
        try:
            msg = choice.message
            if hasattr(msg, "model_extra") and msg.model_extra:
                reasoning_content = msg.model_extra.get("reasoning_content", "") or ""
            if not reasoning_content:
                raw = json.loads(response.model_dump_json())
                reasoning_content = raw["choices"][0]["message"].get("reasoning_content", "") or ""
        except Exception:
            pass
        usage = getattr(response, "usage", None)
        usage_data = {}
        if usage is not None:
            try:
                usage_data = usage.model_dump()
            except Exception:
                try:
                    usage_data = dict(usage)
                except Exception:
                    usage_data = {}
        self.last_call_info = {
            "elapsed": elapsed,
            "max_tokens": max_tokens,
            "finish_reason": getattr(choice, "finish_reason", ""),
            "content_chars": len(response_text),
            "reasoning_chars": len(reasoning_content),
            "has_reasoning_content": bool(reasoning_content.strip()),
            "thinking_type": self.thinking_mode if self.thinking_mode != "default" else "not_set",
            "reasoning_effort": self.reasoning_effort if self.reasoning_effort != "default" else "not_set",
            "enable_thinking": self.enable_thinking if self.enable_thinking != "default" else "not_set",
            "usage": usage_data,
        }
        if not response_text.strip():
            print(
                f"[VLM EMPTY] finish_reason={getattr(choice, 'finish_reason', '')} "
                f"reasoning_chars={len(reasoning_content)} max_tokens={max_tokens}"
            )

        return response_text, reasoning_content
