"""Render prompt templates from ``templates/`` with Jinja2.

``render_prompt`` is the only entry point. Callers pass a template name and the
template variables; template files are read here and nowhere else.

Blank lines: ``trim_blocks`` and ``lstrip_blocks`` drop the line a block tag
stands on, so a template stays readable without leaking indentation into the
prompt. ``keep_trailing_newline`` keeps the newline the file ends with, and
``render_prompt`` normalises the tail to exactly one newline so prompt text does
not depend on how many blank lines a template file happens to end with.

``StrictUndefined`` makes a missing variable raise instead of rendering as an
empty string, so an unfilled placeholder cannot reach the model. Prompt text is
not HTML, so autoescaping stays off.
"""

from __future__ import annotations

from jinja2 import Environment, PackageLoader, StrictUndefined

_ENVIRONMENT = Environment(
    loader=PackageLoader("planner.adapters.prompts", "templates"),
    trim_blocks=True,
    lstrip_blocks=True,
    keep_trailing_newline=True,
    undefined=StrictUndefined,
    autoescape=False,
)


def render_prompt(name: str, /, **values: object) -> str:
    """Render ``templates/<name>.j2`` with ``values`` and one trailing newline."""
    return _ENVIRONMENT.get_template(f"{name}.j2").render(**values).rstrip() + "\n"
