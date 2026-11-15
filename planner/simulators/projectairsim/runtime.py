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


def build_session(*, config, map_entry, family, resources, log):
    """Build the UE5 mission session for one map.

    Called through ``registry.build_session``. The selected catalog entry is the
    single source of the scene JSONC and its config directory, so the same map that
    the process driver launches is the map the session loads -- this used to live in
    ``server_bootstrap``, which meant the HTTP assembly knew ProjectAirSim's scene
    layout while the UE4 side knew nothing of the kind.
    """
    from dataclasses import replace

    from .session import ProjectAirSimMissionSession

    session_config = replace(
        config,
        projectairsim=replace(
            config.projectairsim,
            scene_file=map_entry.scene_file,
            sim_config_path=map_entry.sim_config_path,
        ),
    )
    return ProjectAirSimMissionSession(
        session_config,
        log=log,
        frame_sink=resources.frame_sink,
    )


def describe(vehicles: tuple[str, ...] = VEHICLES) -> SimulatorPorts:
    return SimulatorPorts.from_slots(
        PORT_SLOTS,
        vehicles=vehicles,
        view_modes=VIEW_MODES,
        weather_fields=weather_fields(),
        weather_presets=tuple(preset.id for preset in WEATHER_CATALOG.presets),
        enabled_tasks=ENABLED_TASKS,
    )
