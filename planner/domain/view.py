"""Streaming-view catalog for the dual-vehicle scene.

ProjectAirSim exposes ``World.switch_streaming_view()``, which cycles the main
view through the robots' sensors that are marked ``streaming-enabled`` in
their robot JSONC. There is no free spectator camera API, so a view change is
a number of forward steps around that fixed cycle.

The four streaming cameras are:

    0 Rover1 FrontCamera   -> ``rover_front``
    1 Rover1 Chase         -> ``rover_chase``
    2 Drone1 FrontCamera   -> ``drone_front``
    3 Drone1 Chase         -> ``drone_chase``

``VIEW_MODES`` is the *fallback* order only. The server's real cycle order and
its starting position are not stable across builds, maps or scene loads, so
``planner/adapters/projectairsim/streaming_views.py`` measures them at session
startup by matching xpra screenshots against camera frames and the session
uses that measured order. ``INITIAL_STREAM_INDEX`` applies only when that
calibration fails.
"""

from __future__ import annotations


VIEW_MODES: tuple[str, ...] = (
    "rover_front",
    "rover_chase",
    "drone_front",
    "drone_chase",
)

VIEW_LABELS: dict[str, str] = {
    "rover_front": "无人车前视",
    "rover_chase": "无人车追尾",
    "drone_front": "无人机前视",
    "drone_chase": "无人机追尾",
}

# Explicit front-view selection (console "车前"/"机前" buttons).
VEHICLE_FRONT_VIEW: dict[str, str] = {
    "rover": "rover_front",
    "drone": "drone_front",
}

# The view shown at session start and whenever a vehicle is activated: the
# chase (tail) camera, so the vehicle itself stays in frame.
VEHICLE_DEFAULT_VIEW: dict[str, str] = {
    "rover": "rover_chase",
    "drone": "drone_chase",
}

INITIAL_STREAM_INDEX = 0


def front_view_for(vehicle: object) -> str:
    """Return the streaming view that faces ``vehicle`` forward."""
    if not isinstance(vehicle, str) or vehicle not in VEHICLE_FRONT_VIEW:
        raise ValueError(f"unknown vehicle: {vehicle!r}")
    return VEHICLE_FRONT_VIEW[vehicle]


def default_view_for(vehicle: object) -> str:
    """Return the streaming view a vehicle is shown on when activated."""
    if not isinstance(vehicle, str) or vehicle not in VEHICLE_DEFAULT_VIEW:
        raise ValueError(f"unknown vehicle: {vehicle!r}")
    return VEHICLE_DEFAULT_VIEW[vehicle]


def validate_view_mode(mode: object) -> str:
    if not isinstance(mode, str) or mode not in VIEW_MODES:
        raise ValueError(f"unknown view mode: {mode!r}")
    return mode


def view_index(mode: str) -> int:
    return VIEW_MODES.index(validate_view_mode(mode))


def cycle_delta(current: int, target: int) -> int:
    """Forward ``switch_streaming_view`` steps from ``current`` to ``target``."""
    if not isinstance(current, int) or not isinstance(target, int):
        raise ValueError("streaming view indices must be integers")
    if not 0 <= current < len(VIEW_MODES) or not 0 <= target < len(VIEW_MODES):
        raise ValueError("streaming view index out of range")
    return (target - current) % len(VIEW_MODES)
