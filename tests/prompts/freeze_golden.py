"""Write ``golden.json`` from the prompt builders that ship today.

    python tests/prompts/freeze_golden.py

The file is a JSON object keyed by scenario id; the value is the complete
prompt text that builder produced. Re-running the script rewrites the same
bytes, because the scenarios live in ``scenarios.py`` and take no external
state. Only run it while the pinned text is still the intended text: a diff in
golden.json means the prompt changed.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GOLDEN_PATH = Path(__file__).with_name("golden.json")

sys.path.insert(0, str(ROOT))

from tests.prompts.scenarios import CASES  # noqa: E402


def write_golden(path: Path = GOLDEN_PATH) -> None:
    """Serialise every scenario, sorted by id so the file is stable."""
    payload = json.dumps(CASES, ensure_ascii=False, indent=2, sort_keys=True)
    path.write_text(payload + "\n", encoding="utf-8")


if __name__ == "__main__":
    write_golden()
    print(f"{len(CASES)} scenarios -> {GOLDEN_PATH}")
