from __future__ import annotations

import json
import re
from typing import Any

from planner.errors import ProtocolError


def _reject_constant(value):
    raise ValueError(f"non-finite value: {value}")


def parse_json_object(text: str) -> dict[str, Any]:
    cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", str(text or "").strip(), flags=re.IGNORECASE)
    try:
        def unique_fields(pairs):
            result = {}
            for key, item in pairs:
                if key in result:
                    raise ValueError(f"duplicate field: {key}")
                result[key] = item
            return result
        value = json.loads(cleaned, object_pairs_hook=unique_fields,
                           parse_constant=_reject_constant)
    except (json.JSONDecodeError, ValueError) as exc:
        raise ProtocolError(f"task parser returned invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ProtocolError("task parser response must be a JSON object")
    return value

