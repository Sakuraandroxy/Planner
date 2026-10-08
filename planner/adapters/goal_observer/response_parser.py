import json
import math

from planner.domain.visual_goal import GoalObservation, GoalVisibility
from planner.errors import ProtocolError


def _unique_pairs(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate field: {key}")
        value[key] = item
    return value


def _numbers(raw, size):
    if not isinstance(raw, list) or len(raw) != size:
        raise ValueError("invalid coordinate dimensions")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
           not math.isfinite(v) or not 0 <= v <= 1 for v in raw):
        raise ValueError("coordinates must be finite normalized numbers")
    return tuple(float(v) for v in raw)


def parse_goal_observation(text: str) -> GoalObservation:
    try:
        value = json.loads(text, object_pairs_hook=_unique_pairs)
        keys = {"protocol_version", "status", "bbox", "surface_point", "description", "evidence", "same_target"}
        if not isinstance(value, dict) or set(value) != keys:
            raise ValueError("goal observation fields do not match schema")
        if value["protocol_version"] != "goal_observation_v1":
            raise ValueError("unsupported goal observation protocol")
        status = GoalVisibility(value["status"])
        for name in ("description", "evidence"):
            if not isinstance(value[name], str) or not value[name].strip():
                raise ValueError(f"{name} must be non-empty text")
        if not isinstance(value["same_target"], bool):
            raise ValueError("same_target must be boolean")
        bbox = point = None
        if status is GoalVisibility.VISIBLE:
            bbox, point = _numbers(value["bbox"], 4), _numbers(value["surface_point"], 2)
            if not (bbox[0] < bbox[2] and bbox[1] < bbox[3] and
                    bbox[0] <= point[0] <= bbox[2] and bbox[1] <= point[1] <= bbox[3]):
                raise ValueError("surface point must lie inside a nonempty target box")
        elif value["bbox"] is not None or value["surface_point"] is not None:
            raise ValueError("invisible/ambiguous target cannot have coordinates")
        return GoalObservation(status, bbox, point, value["description"], value["evidence"], value["same_target"])
    except (ValueError, TypeError, KeyError) as exc:
        raise ProtocolError(f"invalid goal observation: {exc}") from exc
