"""Generation fields for explicitly selected HTTP dialects; no URL/model guessing."""


def generation_options(token_limit, reasoning_effort, request_format):
    if request_format not in ("openai", "deepseek"):
        raise ValueError("request_format must be openai or deepseek")
    if request_format == "deepseek":
        options = {"max_tokens": token_limit or 1024,
                   "thinking": {"type": "disabled" if reasoning_effort == "none" else "enabled"}}
        if reasoning_effort is not None and reasoning_effort != "none":
            options["reasoning_effort"] = reasoning_effort
        return options
    if token_limit is None and reasoning_effort is None:
        return {"temperature": 0.0, "max_tokens": 1024}
    options = {"max_completion_tokens": token_limit or 1024}
    if reasoning_effort is not None:
        options["reasoning_effort"] = reasoning_effort
    return options
