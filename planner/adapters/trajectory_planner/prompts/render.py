"""Load prompt templates from ``templates/`` and fill ``${name}`` placeholders.

Fragments that are embedded in a larger prompt are read with ``load_fragment``:
it drops the single newline that ends the template file, so the composer in
``base.py``/``free_navigation.py``/``visual_goal_navigation.py`` keeps control
of every separator.
"""

from pathlib import Path

from planner.adapters.prompt_render import render

TEMPLATES_DIR = Path(__file__).with_name("templates")


def load(name: str) -> str:
    """Read ``templates/<name>.md`` verbatim, including its line endings."""
    return (TEMPLATES_DIR / f"{name}.md").read_text(encoding="utf-8")


def load_fragment(name: str) -> str:
    """Read a template that is composed into a larger prompt, minus its final newline."""
    return load(name).removesuffix("\n")
