"""ProjectAirSim (UE5) simulator: which ports it fills, and how to describe them.

``PORT_SLOTS`` mirrors what ``session.py`` already wires together. ``visual_goal_above``
is absent because there is no ``camera_view``: ProjectAirSim exposes
``World.switch_streaming_view()`` for a preset camera cycle and no way to write a
body-mounted camera pose, so the task cannot be offered here regardless of what
any config file says.
"""

from __future__ import annotations

from ...domain.view import VIEW_MODES
from ...domain.weather import WEATHER_CATALOG, WEATHER_FIELDS
from ...simulator.ports import SimulatorPorts

PORT_SLOTS: tuple[str, ...] = (
    "vehicle",
    "observation",
    "streaming",
    "controlled_capture",
    "moving_capture",
    "weather",
    "views",
    "models",
    "frames",
    "telemetry",
    "home",
)

#: Empty means every task whose ports are present may be offered. UE5 has been
#: flown with all three of its task kinds, so nothing is withheld here.
#: ``visual_goal_above`` still drops out, because ``camera_view`` is not in
#: PORT_SLOTS. That is the design point: the gap is a missing interface, not a
#: disabled feature flag.
ENABLED_TASKS: tuple[str, ...] = ()

VEHICLES: tuple[str, ...] = ("rover", "drone")


def weather_fields() -> dict[str, str]:
    """Canonical to simulator spelling. UE5 names match the planner exactly."""
    return {name: name for name in WEATHER_FIELDS}


def describe(vehicles: tuple[str, ...] = VEHICLES) -> SimulatorPorts:
    return SimulatorPorts.from_slots(
        PORT_SLOTS,
        vehicles=vehicles,
        view_modes=VIEW_MODES,
        weather_fields=weather_fields(),
        weather_presets=tuple(preset.id for preset in WEATHER_CATALOG.presets),
        enabled_tasks=ENABLED_TASKS,
    )
