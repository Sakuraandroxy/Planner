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


def validate_mission_envelope(data: dict[str, Any]) -> None:
    """Explicit rejection and executable plans are mutually exclusive."""
    if not isinstance(data, dict):
        raise ProtocolError("task parser response must be a JSON object")
    if set(data)-{"protocol_version", "stages", "error"}:
        raise ProtocolError("unknown mission response fields")
    if data.get("protocol_version") != "mission_plan_v1":
        raise ProtocolError("unsupported mission protocol")
    stages = data.get("stages")
    if not isinstance(stages, list):
        raise ProtocolError("task parser response has no stages list")
    if "error" in data:
        error = data["error"]
        if stages or not isinstance(error, str) or not error.strip():
            raise ProtocolError("mission rejection requires empty stages and a non-empty error")
        raise ProtocolError(f"mission rejected by task parser: {error.strip()}")
    if not stages:
        raise ProtocolError("mission contains no stages")

