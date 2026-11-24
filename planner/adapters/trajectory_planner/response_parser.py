"""Parse the trajectory model's JSON waypoint array."""

from __future__ import annotations

import json
import re

from ...domain.pose import RelativePoseDelta
from ...domain.trajectory import RelativeTrajectory
from ...errors import ProtocolError

_ARRAY_PATTERN = re.compile(r"\[\s*\[.*?\]\s*\]", re.DOTALL)


def _json_array(text: str):
    """Return the outermost JSON array found in ``text``."""
    match = _ARRAY_PATTERN.search(str(text or ""))
    if not match:
        raise ProtocolError("trajectory planner returned no JSON array")
    try:
        return json.loads(match.group(0))
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"trajectory planner returned invalid JSON: {exc}") from exc


def parse_trajectory(text: str, expected_points: int = 5) -> RelativeTrajectory:
    value = _json_array(text)
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


def parse_translation_trajectory(text: str) -> RelativeTrajectory:
    """Parse the bare pipeline's ``[dx, dy, dz]`` array; yaw belongs to the engine.

    The count is not fixed: the model returns as many waypoints as the instruction
    needs, and the workflow decides how many to fly. Every parsed point carries a
    zero yaw delta because this protocol never asks the model for a heading.
    """
    value = _json_array(text)
    if not isinstance(value, list) or not value:
        raise ProtocolError("translation planner must return at least one point")
    points = []
    for index, raw in enumerate(value):
        if not isinstance(raw, list) or len(raw) != 3:
            raise ProtocolError(f"translation point {index} must be [dx, dy, dz]")
        if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in raw):
            raise ProtocolError(
                f"translation point {index} must contain JSON numbers, not strings/booleans"
            )
        try:
            dx, dy, dz = (float(item) for item in raw)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ProtocolError(f"translation point {index} is invalid: {exc}") from exc
        points.append(RelativePoseDelta(dx=dx, dy=dy, dz=dz, dyaw_deg=0.0))
    return RelativeTrajectory(tuple(points))