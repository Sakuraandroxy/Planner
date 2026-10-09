import json
import math

from planner.domain.visual_goal import GoalVisibility
from planner.domain.target_surface import GoalSurfaceObservation, SurfaceCandidate
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


def parse_goal_observation(text: str, maximum_candidates: int = 5) -> GoalSurfaceObservation:
    try:
        value = json.loads(text, object_pairs_hook=_unique_pairs)
        keys = {"protocol_version", "status", "bbox", "surface_candidates", "description", "evidence", "same_target"}
        if not isinstance(value, dict) or set(value) != keys:
            raise ValueError("goal observation fields do not match schema")
        if value["protocol_version"] != "goal_observation_v2":
            raise ValueError("unsupported goal observation protocol")
        status = GoalVisibility(value["status"])
        for name in ("description", "evidence"):
            if not isinstance(value[name], str) or not value[name].strip():
                raise ValueError(f"{name} must be non-empty text")
        if not isinstance(value["same_target"], bool):
            raise ValueError("same_target must be boolean")
        bbox = None
        candidates = []
        raw_candidates = value["surface_candidates"]
        if not isinstance(raw_candidates, list):
            raise ValueError("surface_candidates must be a list")
        if status is GoalVisibility.VISIBLE:
            bbox = _numbers(value["bbox"], 4)
            if not (bbox[0] < bbox[2] and bbox[1] < bbox[3]):
                raise ValueError("target box must be nonempty")
            if not 1 <= len(raw_candidates) <= maximum_candidates:
                raise ValueError(f"visible target requires 1 to {maximum_candidates} candidates")
            ids, points = set(), set()
            for raw in raw_candidates:
                if not isinstance(raw, dict) or set(raw) != {
                    "candidate_id", "point", "description", "matches_reference_region"
                }:
                    raise ValueError("surface candidate fields do not match schema")
                for field in ("candidate_id", "description"):
                    if not isinstance(raw[field], str) or not raw[field].strip():
                        raise ValueError(f"candidate {field} must be non-empty text")
                point = _numbers(raw["point"], 2)
                if not bbox[0] <= point[0] <= bbox[2] or not bbox[1] <= point[1] <= bbox[3]:
                    raise ValueError("surface point must lie inside target box")
                if raw["candidate_id"] in ids or point in points:
                    raise ValueError("duplicate candidate ID or point")
                if not isinstance(raw["matches_reference_region"], bool):
                    raise ValueError("matches_reference_region must be boolean")
                ids.add(raw["candidate_id"])
                points.add(point)
                candidates.append(SurfaceCandidate(raw["candidate_id"], point, raw["description"],
                                                   raw["matches_reference_region"]))
            if sum(item.matches_reference_region for item in candidates) > 1:
                raise ValueError("only one candidate can claim the selected reference location")
        elif value["bbox"] is not None or raw_candidates:
            raise ValueError("invisible/ambiguous target cannot have coordinates")
        return GoalSurfaceObservation(status, bbox, tuple(candidates), value["description"],
                                      value["evidence"], value["same_target"])
    except (ValueError, TypeError, KeyError) as exc:
        raise ProtocolError(f"invalid goal observation: {exc}") from exc
