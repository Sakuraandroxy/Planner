from __future__ import annotations

import json
import re

from planner.domain.pose import RelativePoseDelta
from planner.domain.trajectory import RelativeTrajectory
from planner.errors import ProtocolError


def parse_trajectory(text: str, expected_points: int = 5) -> RelativeTrajectory:
    match = re.search(r"\[\s*\[.*?\]\s*\]", str(text or ""), re.DOTALL)
    if not match:
        raise ProtocolError("trajectory planner returned no JSON array")
    try:
        value = json.loads(match.group(0))
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"trajectory planner returned invalid JSON: {exc}") from exc
    if not isinstance(value, list) or len(value) != expected_points:
        raise ProtocolError(f"trajectory planner must return exactly {expected_points} points")
    points = []
    for index, raw in enumerate(value):
        if not isinstance(raw, list) or len(raw) != 4:
            raise ProtocolError(f"trajectory point {index} must be [dx, dy, dz, dyaw_deg]")
        try:
            points.append(RelativePoseDelta(*(float(item) for item in raw)))
        except (TypeError, ValueError) as exc:
            raise ProtocolError(f"trajectory point {index} is invalid: {exc}") from exc
    return RelativeTrajectory(tuple(points))

