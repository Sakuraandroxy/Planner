"""Render prompt templates from ``templates/`` with ``${name}`` placeholders.

The prompt text only needs plain value interpolation, so a regular-expression
replacement is enough and no template engine is required. A placeholder
without a supplied value raises ``KeyError`` instead of reaching the model
verbatim.
"""

import re
from pathlib import Path

TEMPLATES_DIR = Path(__file__).with_name("templates")

_PLACEHOLDER = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")


def load(name: str) -> str:
    """Read ``templates/<name>.md`` verbatim, including its line endings."""
    return (TEMPLATES_DIR / f"{name}.md").read_text(encoding="utf-8")


def render(template: str, **values: object) -> str:
    """Replace every ``${name}`` in ``template`` with ``str(values[name])``."""

    def substitute(match: re.Match[str]) -> str:
        key = match.group(1)
        if key not in values:
            raise KeyError(f"prompt template has no value for ${{{key}}}")
        return str(values[key])

    return _PLACEHOLDER.sub(substitute, template)
