"""Optional backend configuration; presence never activates detection."""
from dataclasses import dataclass, fields
import os
from .validation import mapping, string, number, integer


@dataclass(frozen=True)
class TargetDetectionConfig:
    url: str = "https://api.deepseek.com"
    model: str = "deepseek-flash"
    api_key: str = ""
    timeout_s: float = 20.0
    reasoning_effort: str = "none"
    request_format: str = "deepseek"
    max_completion_tokens: int = 64
    max_image_side: int = 512


def target_detection_config(raw):
    mapping(raw, "target_detection", {f.name for f in fields(TargetDetectionConfig)} | {"api_key_env"})
    options = {k: v for k, v in raw.items() if k != "api_key_env"}
    env = string(raw.get("api_key_env", "TARGET_DETECTION_API_KEY"), "target_detection.api_key_env", allow_empty=True)
    options["api_key"] = (os.environ.get(env, "") if env else "") or options.get("api_key", "")
    config = TargetDetectionConfig(**options)
    for name in ("url", "model", "api_key"):
        string(getattr(config, name), "target_detection."+name, allow_empty=name == "api_key")
    number(config.timeout_s, "target_detection.timeout_s")
    integer(config.max_completion_tokens, "target_detection.max_completion_tokens", 1, 32768)
    integer(config.max_image_side, "target_detection.max_image_side", 32, 8192)
    if config.reasoning_effort not in {"none", "low", "medium", "high"}:
        raise ValueError("invalid target_detection.reasoning_effort")
    if config.request_format not in ("openai", "deepseek"):
        raise ValueError("request_format must be openai or deepseek")
    return config
