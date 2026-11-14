from __future__ import annotations

import json
import re
from typing import Any

from planner.errors import ProtocolError


def parse_json_object(text: str) -> dict[str, Any]:
    cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", str(text or "").strip(), flags=re.IGNORECASE)
    try:
        value = json.loads(cleaned)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"task parser returned invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ProtocolError("task parser response must be a JSON object")
    return value

