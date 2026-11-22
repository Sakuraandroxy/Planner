"""AirSim (UE4) simulator: which ports it fills, and how to build them.

``PORT_SLOTS`` is what the console is told before any simulator is running.
``build_ports`` is what the mission actually uses.
``tests/test_architecture.py::test_airsim_slots_are_backed_by_real_code`` checks
that every declared slot is a real bundle field and that the three vehicle classes
carry the port methods, so the advertised flags cannot drift from the code.
"""

from __future__ import annotations

from ...simulator import SimulatorPorts

#: Slots AirSim implements. Absent slots are capabilities UE4 does not offer:
#: no streaming camera cycle, no appearance models, no frame stream, and no
#: return-home route implemented here. Trajectory telemetry is not in this list
#: at all -- the remaining-path read is a service feature, not a simulator
#: permission, so it is offered to every family.
PORT_SLOTS: tuple[str, ...] = (
    "vehicle",
    "observation",
    "streaming",
    "moving_capture",
    "weather",
)

VEHICLES: tuple[str, ...] = ("drone",)


def build_ports(
    *,
    vehicle,
    observation,
    weather,
    moving_capture=None,
) -> SimulatorPorts:
    """Bundle the live AirSim objects into the ports the mission uses.

    Called by :class:`planner.simulators.airsim.session.AirSimMissionSession`, which
    is also what makes ``describe()`` honest: the slots advertised before a
    simulator is running are the slots this function can fill once it is running.
    ``views``, ``models``, ``frames`` and ``home`` stay empty because UE4 does not
    implement them, and the console hides those controls from that fact. The
    trajectory read is not a port: ``/api/trajectory`` serves the service's own
    hub for every family.
    """
    return SimulatorPorts(
        vehicle=vehicle,
        observation=observation,
        streaming=vehicle,                 # AirSimVehicle implements stream_command
        moving_capture=moving_capture if moving_capture is not None else observation,
        weather=weather,
        vehicles=VEHICLES,
    )


def build_session(*, config, map_entry, family, resources, log):
    """Build the UE4 mission session for one map.

    Called through ``registry.build_session``. ``map_entry`` is part of the uniform
    builder contract; UE4 does not read it, because its scene and spawn come from
    the engine's own ``settings.json`` rather than from a planner scene JSONC. What
    it does need -- the RPC port, the vehicle name and the camera name -- is family
    data, and the family was resolved from this map's ``simulator`` field.

    ``resources`` is not forwarded to the session. The registry hands it to
    ``assemble_missions`` once this session has reported its ports, so the
    simulator package never builds a planner, a detector or a telemetry sink.
    """
    from .session import AirSimMissionSession

    return AirSimMissionSession(config, family, log=log)


def describe() -> SimulatorPorts:
    """The declared bundle, for capability reporting without a live simulator."""
    return SimulatorPorts.from_slots(
        PORT_SLOTS,
        vehicles=VEHICLES,
        view_modes=(),
        weather_fields=WEATHER_FIELDS,
        weather_presets=WEATHER_PRESETS,
    )


#: Canonical name -> AirSim ``WeatherParameter`` spelling. Only the road wetness
#: differs from the planner's canonical name; it lives here rather than in
#: config because this module is what talks to AirSim.
WEATHER_FIELDS: dict[str, str] = {
    "rain": "rain",
    "road_wetness": "roadwetness",
    "snow": "snow",
    "road_snow": "road_snow",
    "maple_leaf": "maple_leaf",
    "road_leaf": "road_leaf",
    "dust": "dust",
    "fog": "fog",
}

WEATHER_PRESETS: tuple[str, ...] = ("clear", "rain", "fog", "snow")
