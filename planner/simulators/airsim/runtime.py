"""AirSim (UE4) simulator: which ports it fills, and how to build them.

``PORT_SLOTS`` is what the console is told before any simulator is running.
``build_ports`` is what the mission actually uses. ``tests/test_simulator_ports.py``
asserts the two agree, so the advertised flags cannot drift from the code.
"""

from __future__ import annotations

from ...simulator.ports import SimulatorPorts

#: Slots AirSim implements. Absent slots are capabilities UE4 does not offer:
#: no streaming camera cycle, no appearance models, no frame stream, no
#: trajectory telemetry, and no return-home (the old planner had no such route).
PORT_SLOTS: tuple[str, ...] = (
    "vehicle",
    "observation",
    "streaming",
    "moving_capture",
    "camera_view",
    "weather",
)

#: AirSim was only ever flown on these tasks. ``free_navigation`` is technically
#: reachable (the vehicle streams velocity), but the planner behind it is new in
#: main and has never run against AirSim depth, so it stays off the offer list.
ENABLED_TASKS: tuple[str, ...] = (
    "basic_motion",
    "visual_goal_navigation",
    "visual_goal_above",
)

VEHICLES: tuple[str, ...] = ("drone",)


def describe() -> SimulatorPorts:
    """The declared bundle, for capability reporting without a live simulator."""
    return SimulatorPorts.from_slots(
        PORT_SLOTS,
        vehicles=VEHICLES,
        view_modes=(),
        weather_fields=WEATHER_FIELDS,
        weather_presets=WEATHER_PRESETS,
        enabled_tasks=ENABLED_TASKS,
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
