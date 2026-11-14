"""Independent semantic-completion model configuration."""
from dataclasses import dataclass, fields
import math
import os

@dataclass(frozen=True)
class TaskCompletionConfig:
    url: str = "https://api.deepseek.com"
    model: str = "deepseek-flash"
    api_key: str = ""
    timeout_s: float = 20.0
    max_completion_tokens: int = 2048
    reasoning_effort: str = "none"
    request_format: str = "deepseek"


def completion_config(raw):
    if not isinstance(raw, dict):
        raise ValueError("task_completion must be a mapping")
    names = {f.name for f in fields(TaskCompletionConfig)}
    if set(raw)-names-{"api_key_env"}:
        raise ValueError("unknown task_completion settings")
    options = {key: value for key, value in raw.items() if key in names}
    env = raw.get("api_key_env", "TASK_COMPLETION_API_KEY")
    if not isinstance(env, str):
        raise ValueError("task_completion.api_key_env must be text")
    options["api_key"] = (os.environ.get(env, "") if env else "") or options.get("api_key", "")
    config = TaskCompletionConfig(**options)
    if any(not isinstance(value, str) for value in (config.url, config.model, config.api_key)):
        raise ValueError("completion URL, model and key must be text")
    if not config.url.strip() or not config.model.strip():
        raise ValueError("completion URL and model are required")
    if isinstance(config.timeout_s, bool) or not isinstance(config.timeout_s, (int, float)) or not math.isfinite(config.timeout_s) or config.timeout_s <= 0:
        raise ValueError("completion timeout must be finite and positive")
    if isinstance(config.max_completion_tokens, bool) or not isinstance(config.max_completion_tokens, int) or config.max_completion_tokens < 1:
        raise ValueError("max_completion_tokens must be a positive integer")
    if config.reasoning_effort not in {"none", "low", "medium", "high"}:
        raise ValueError("invalid completion reasoning_effort")
    if config.request_format not in ("openai", "deepseek"):
        raise ValueError("request_format must be openai or deepseek")
    return config


