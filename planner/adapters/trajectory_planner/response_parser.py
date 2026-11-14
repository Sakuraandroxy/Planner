"""Parse the trajectory model's JSON waypoint array."""

from __future__ import annotations

import json
import re

from ...domain.pose import RelativePoseDelta
from ...domain.trajectory import RelativeTrajectory
from ...errors import ProtocolError

_ARRAY_PATTERN = re.compile(r"\[\s*\[.*?\]\s*\]", re.DOTALL)


def parse_trajectory(text: str, expected_points: int = 5) -> RelativeTrajectory:
    match = _ARRAY_PATTERN.search(str(text or ""))
    if not match:
        raise ProtocolError("trajectory planner returned no JSON array")
    try:
        value = json.loads(match.group(0))
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"trajectory planner returned invalid JSON: {exc}") from exc
    if not isinstance(value, list) or not 1 <= len(value) <= expected_points:
        raise ProtocolError(
            f"trajectory planner must return 1 to {expected_points} points; "
            f"received {len(value) if isinstance(value, list) else type(value).__name__}"
        )
    points = []
    for index, raw in enumerate(value):
        if not isinstance(raw, list) or len(raw) != 4:
            raise ProtocolError(f"trajectory point {index} must be [dx, dy, dz, dyaw_deg]")
        if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in raw):
            raise ProtocolError(
                f"trajectory point {index} must contain JSON numbers, not strings/booleans"
            )
        try:
            points.append(RelativePoseDelta(*(float(item) for item in raw)))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ProtocolError(f"trajectory point {index} is invalid: {exc}") from exc
    return RelativeTrajectory(tuple(points))