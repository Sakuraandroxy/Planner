"""Physically reachable goal selection for 3DG-VLN episode metadata."""

from __future__ import annotations

from typing import Any, Optional


def _point3(value: Any) -> Optional[list[float]]:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except (TypeError, ValueError):
        return None


def object_target_position(mark: dict) -> list[float]:
    target = mark.get("target", {}) if isinstance(mark, dict) else {}
    value = target.get("position") if isinstance(target, dict) else target
    return _point3(value) or [0.0, 0.0, 0.0]


def success_goal_position(mark: dict, reference: str = "end") -> list[float]:
    """Select a reachable UAV goal while retaining centroid compatibility.

    3DG-VLN ``end`` is the demonstrated UAV endpoint, whereas
    ``target.position`` is the referenced object's origin/center.  The latter
    can lie inside a car or building and must not control physical termination.
    """
    normalized = str(reference or "end").strip().lower()
    if normalized in {"end", "endpoint", "demonstration_end", "safe_end"}:
        endpoint = _point3(mark.get("end") if isinstance(mark, dict) else None)
        if endpoint is not None:
            return endpoint
    return object_target_position(mark)
