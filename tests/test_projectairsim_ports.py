"""Every slot ProjectAirSim declares must be backed by an implementation.

UE5 has no ``build_ports``: the console learns its capabilities from ``PORT_SLOTS``
before a session exists, and the assembly builds the small bundle it needs out of
``MissionInputs`` at wiring time. So the declared list has nothing to compare itself
to unless this test names the object behind each slot. It exists because
``path_follow`` was advertised here for a while with nothing behind it: the slot was
declared, ``mission_inputs`` did not hand a follower over, and the bare pipeline was
unreachable until that handoff was fixed.

``controlled_capture`` is the one declared slot with no implementation: the package
has no ``request_capture``. That gap predates this branch
(``reports/PORT_PARITY.md``, 顺带发现; ``HANDOFF.md`` §10), so the test pins it as the
only gap -- a second one fails here instead of being found on the machine.
"""

from planner.application.server_controller import ServerController
from planner.simulator.ports import PIPELINE_REQUIREMENTS, SimulatorPorts
from planner.simulators.projectairsim import runtime as pas_runtime
from tests.test_projectairsim_drone import make_drone
from tests.test_projectairsim_rover import make_rover
from tests.test_projectairsim_session import make_session

#: Declared slot -> the object that answers it, and the callables that object has
#: to carry. ``weather``/``views``/``home`` are served by the session itself and
#: ``models`` by the controller's scene-reload path; the rest by the vehicles.
_SLOT_OWNERS = {
    "vehicle": "vehicle",
    "observation": "vehicle",
    "streaming": "vehicle",
    "moving_capture": "vehicle",
    "path_follow": "path_follow",
    "weather": "session",
    "views": "session",
    "home": "session",
    "frames": "session",
}

_SLOT_CALLABLES = {
    "vehicle": ("current_pose", "execute_segment", "cancel", "hover", "collision_state"),
    "observation": ("capture",),
    "streaming": ("stream_state", "stream_velocity"),
    "moving_capture": ("capture_moving",),
    "path_follow": ("follow_path",),
    "weather": ("set_weather", "weather_catalog"),
    # The readable side of views is a property (``view_mode``), so only the
    # setter can be probed as a callable here.
    "views": ("set_view_mode",),
    "home": ("return_home",),
    "frames": ("_sink_for",),
}


def _live_parts():
    """A started drone, a started rover, and the session that routes to them."""
    drone, _, _, _ = make_drone()
    rover, _, _ = make_rover()
    drone.start()
    rover.start()
    session = make_session(active="drone")
    session._drone, session._rover = drone, rover
    return drone, rover, session


def test_every_declared_slot_has_an_implementation_behind_it():
    drone, _rover, session = _live_parts()
    try:
        owners = {
            "vehicle": drone,
            "session": session,
            "path_follow": session.mission_inputs().path_follow,
        }
        backed = {
            slot
            for slot, owner_name in _SLOT_OWNERS.items()
            if all(
                callable(getattr(owners[owner_name], name, None))
                for name in _SLOT_CALLABLES[slot]
            )
        }
        # ``models`` is applied by reloading the scene from the controller, not by
        # an object handed to the assembly, so it is checked where it lives.
        if callable(getattr(ServerController, "request_models", None)):
            backed.add("models")
        declared = set(pas_runtime.PORT_SLOTS)
        assert declared - backed == {"controlled_capture"}, (
            "PORT_SLOTS advertises a slot with no implementation behind it; "
            f"declared={sorted(declared)} backed={sorted(backed)}"
        )
    finally:
        drone.close()
        _rover.close()


def test_the_bare_bundle_built_from_live_objects_has_no_missing_slot():
    """The bundle the assembly builds per arm, made from the real classes."""
    drone, _rover, session = _live_parts()
    try:
        inputs = session.mission_inputs()
        ports = SimulatorPorts(
            vehicle=drone, observation=drone, path_follow=inputs.path_follow
        )
        assert ports.missing(PIPELINE_REQUIREMENTS["bare"]) == ()
        assert ports.missing(PIPELINE_REQUIREMENTS["staged"]) == ()
        # The router the assembly receives is what the workflow calls; it has to
        # resolve to the session's active vehicle rather than a stale object.
        assert callable(inputs.path_follow.follow_path)
    finally:
        drone.close()
        _rover.close()
