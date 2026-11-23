"""ProjectAirSim (UE5) simulator: which ports it fills, and how to describe them.

``PORT_SLOTS`` is what the console is told before a simulator is running;
``build_ports`` is the bundle the live session hands over, and ``verify_slots``
refuses the boot when the two disagree.
"""

from __future__ import annotations

from planner.domain import VIEW_MODES
from planner.domain import WEATHER_CATALOG, WEATHER_FIELDS
from ...simulator import SimulatorPorts

PORT_SLOTS: tuple[str, ...] = (
    "vehicle",
    "observation",
    "streaming",
    "moving_capture",
    "weather",
    "views",
    "models",
    "frames",
    "home",
)

VEHICLES: tuple[str, ...] = ("rover", "drone")


def weather_fields() -> dict[str, str]:
    """Canonical to simulator spelling. UE5 names match the planner exactly."""
    return {name: name for name in WEATHER_FIELDS}


def build_ports(
    *,
    vehicle,
    observation,
    streaming,
    moving_capture,
    weather,
    views,
    models,
    frames,
    home,
) -> SimulatorPorts:
    """Bundle the live UE5 objects into the ports the mission uses.

    Each slot holds the object that already serves that feature here, not a wrapper
    around it: the vehicle adapters carry capture, streaming and moving capture; the
    session is the weather, view and home control; the appearance catalog is the
    ``/api/models`` source; and ``frames`` is the frame sink the session was given.
    ``controlled_capture`` is not declared because nothing on this engine implements
    ``request_capture`` and no production path calls it.
    """
    ports = SimulatorPorts(
        vehicle=vehicle,
        observation=observation,
        streaming=streaming,
        moving_capture=moving_capture,
        weather=weather,
        views=views,
        models=models,
        frames=frames,
        home=home,
        vehicles=VEHICLES,
    )
    ports.verify_slots(PORT_SLOTS, simulator="projectairsim")
    return ports


def build_session(*, config, map_entry, family, resources, log):
    """Build the UE5 mission session for one map.

    Called through ``registry.build_session``. The selected catalog entry is the
    single source of the scene JSONC and its config directory, so the same map that
    the process driver launches is the map the session loads -- this used to live in
    ``server_bootstrap``, which meant the HTTP assembly knew ProjectAirSim's scene
    layout while the UE4 side knew nothing of the kind.
    """
    from dataclasses import replace

    from ...config import build_camera_settings
    from .session import ProjectAirSimMissionSession

    scene = map_entry.scene
    session_config = replace(
        config,
        projectairsim=replace(
            config.projectairsim,
            scene_file=map_entry.scene_file,
            sim_config_path=map_entry.sim_config_path,
            delay_after_load_s=scene.delay_after_load_s,
            rover_name=scene.rover_name,
            drone_name=scene.drone_name,
            front_camera=scene.front_camera,
            chase_camera=scene.chase_camera,
            down_camera=scene.down_camera,
            uncapturable_views=scene.uncapturable_views,
            view_cycle_start=scene.view_cycle_start,
        ),
        # Camera geometry is a property of the simulator family, not the profile.
        camera=build_camera_settings(family.camera),
    )
    return ProjectAirSimMissionSession(
        session_config,
        log=log,
        frame_sink=resources.frame_sink,
        trajectory=resources.trajectory,
    )


def describe(vehicles: tuple[str, ...] = VEHICLES) -> SimulatorPorts:
    return SimulatorPorts.from_slots(
        PORT_SLOTS,
        vehicles=vehicles,
        view_modes=VIEW_MODES,
        weather_fields=weather_fields(),
        weather_presets=tuple(preset.id for preset in WEATHER_CATALOG.presets),
    )
