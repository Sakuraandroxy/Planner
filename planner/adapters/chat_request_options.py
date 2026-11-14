"""Generation fields for explicitly selected HTTP dialects; no URL/model guessing."""

SUPPORTED_REQUEST_FORMATS = ("openai", "deepseek", "anthropic")


def generation_options(token_limit, reasoning_effort, request_format):
    if request_format not in SUPPORTED_REQUEST_FORMATS:
        raise ValueError(
            "request_format must be openai, deepseek or anthropic"
        )
    if request_format == "deepseek":
        options = {"max_tokens": token_limit or 1024,
                   "thinking": {"type": "disabled" if reasoning_effort == "none" else "enabled"}}
        if reasoning_effort is not None and reasoning_effort != "none":
            options["reasoning_effort"] = reasoning_effort
        return options
    if request_format == "anthropic":
        # The Messages API has no reasoning_effort switch; explicitly disable
        # thinking because proxies (e.g. DashScope) otherwise spend the whole
        # token budget on thinking blocks and return no answer text.
        if reasoning_effort not in (None, "none"):
            raise ValueError(
                "anthropic request format does not support reasoning_effort"
            )
        return {"max_tokens": token_limit or 1024, "temperature": 0.0,
                "thinking": {"type": "disabled"}}
    if token_limit is None and reasoning_effort is None:
        return {"temperature": 0.0, "max_tokens": 1024}
    options = {"max_completion_tokens": token_limit or 1024}
    if reasoning_effort is not None:
        options["reasoning_effort"] = reasoning_effort
    return options