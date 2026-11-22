"""Interpolate ``${name}`` placeholders in prompt templates.

The prompt text only needs plain value interpolation, so a regular-expression
replacement is enough and no template engine is required. A placeholder
without a supplied value raises ``KeyError`` instead of reaching the model
verbatim.
"""

import re

_PLACEHOLDER = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")


def render(template: str, **values: object) -> str:
    """Replace every ``${name}`` in ``template`` with ``str(values[name])``."""

    def substitute(match: re.Match[str]) -> str:
        key = match.group(1)
        if key not in values:
            raise KeyError(f"prompt template has no value for ${{{key}}}")
        return str(values[key])

    return _PLACEHOLDER.sub(substitute, template)
