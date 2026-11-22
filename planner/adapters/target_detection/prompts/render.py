"""Load detection prompt templates from ``templates/``.

``load_fragment`` drops the newline that ends the file, because the detection
messages are composed into one request string.
"""

from pathlib import Path

from planner.adapters.prompt_render import render

TEMPLATES_DIR = Path(__file__).with_name("templates")


def load(name: str) -> str:
    """Read ``templates/<name>.md`` verbatim, including its line endings."""
    return (TEMPLATES_DIR / f"{name}.md").read_text(encoding="utf-8")


def load_fragment(name: str) -> str:
    """Read a template without the newline that ends the file."""
    return load(name).removesuffix("\n")
