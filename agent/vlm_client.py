"""VLMClient - thin wrapper around the OpenAI chat completions API."""
import json
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

    def call(self, messages, max_tokens=8192):
        """Send messages to VLM, return (response_text, reasoning_content)."""
        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=messages,
            max_tokens=max_tokens,
            temperature=self.temperature,
        )
        response_text = response.choices[0].message.content or ""

        reasoning_content = ""
        try:
            msg = response.choices[0].message
            if hasattr(msg, "model_extra") and msg.model_extra:
                reasoning_content = msg.model_extra.get("reasoning_content", "") or ""
            if not reasoning_content:
                raw = json.loads(response.model_dump_json())
                reasoning_content = raw["choices"][0]["message"].get("reasoning_content", "") or ""
        except Exception:
            pass

        return response_text, reasoning_content
