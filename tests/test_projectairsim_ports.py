"""Every slot ProjectAirSim declares must be backed by an implementation.

UE5 has no ``build_ports``: the console learns its capabilities from ``PORT_SLOTS``
before a session exists, and the assembly builds the small bundle it needs out of
``MissionInputs`` at wiring time. So the declared list has nothing to compare itself
to unless this test names the object behind each slot.

The check runs against **each vehicle**, not against the router in
``mission_inputs``: ``_ActivePathFollow`` always has a ``follow_path``, so checking
it would pass even with both vehicles' implementations deleted. A bare task can run
on either arm, so both have to answer the vehicle-side slots.

``controlled_capture`` is the one declared slot with no implementation: the package
has no ``request_capture``. That gap predates this branch (``reports/PORT_PARITY.md``,
顺带发现; ``HANDOFF.md`` §10), so the test pins it as the only gap -- a second one
fails here instead of being found on the machine.

The readable side of ``views`` is a property (``session.view_mode``), so only the
setter is probed as a callable.
"""

from planner.application.server_controller import ServerController
from planner.simulator.ports import PIPELINE_REQUIREMENTS, SimulatorPorts
from planner.simulators.projectairsim import runtime as pas_runtime
from planner.simulators.projectairsim.drone import ProjectAirSimDrone
from planner.simulators.projectairsim.rover import ProjectAirSimRover
from tests.test_projectairsim_drone import make_drone
from tests.test_projectairsim_rover import make_rover
from tests.test_projectairsim_session import make_session

#: Slots each vehicle has to implement on its own.
_VEHICLE_SLOTS = {
    "vehicle": ("current_pose", "execute_segment", "cancel", "hover", "collision_state"),
    "observation": ("capture",),
    "streaming": ("stream_state", "stream_velocity"),
    "moving_capture": ("capture_moving",),
    "path_follow": ("follow_path",),
}

#: Slots the session itself serves (appearance models go through the controller).
_SESSION_SLOTS = {
    "weather": ("set_weather", "weather_catalog"),
    "views": ("set_view_mode",),
    "home": ("return_home",),
    "frames": ("_sink_for",),
}

#: Declared with nothing behind it in this package; see the module docstring.
_NO_IMPLEMENTATION = {"controlled_capture"}


def _live_parts():
    """A started drone, a started rover, and the session that routes to them."""
    drone, _, _, _ = make_drone()
    rover, _, _ = make_rover()
    drone.start()
    rover.start()
    session = make_session(active="drone")
    session._drone, session._rover = drone, rover
    return drone, rover, session


def test_every_declared_slot_is_implemented_by_each_vehicle_or_the_session():
    drone, rover, session = _live_parts()
    try:
        declared = set(pas_runtime.PORT_SLOTS)
        unaccounted = declared - set(_VEHICLE_SLOTS) - set(_SESSION_SLOTS) - {"models"}
        assert unaccounted == _NO_IMPLEMENTATION, (
            f"PORT_SLOTS declares {sorted(unaccounted)} with nothing behind it"
        )
        for name, vehicle in (("drone", drone), ("rover", rover)):
            for slot, methods in _VEHICLE_SLOTS.items():
                assert slot in declared
                missing = [
                    method for method in methods
                    if not callable(getattr(vehicle, method, None))
                ]
                assert not missing, f"{name} backs {slot} without {missing}"
        for slot, methods in _SESSION_SLOTS.items():
            missing = [
                method for method in methods
                if not callable(getattr(session, method, None))
            ]
            assert not missing, f"the session backs {slot} without {missing}"
        # Appearance models are applied by reloading the scene from the controller.
        assert callable(getattr(ServerController, "request_models", None))
    finally:
        drone.close()
        rover.close()


def test_the_path_follower_the_assembly_gets_is_a_router_not_a_vehicle():
    """``mission_inputs`` hands over one object for both arms; the vehicles implement
    the slot, the router only decides which of them the running task drives."""
    drone, rover, session = _live_parts()
    try:
        follower = session.mission_inputs().path_follow
        assert callable(follower.follow_path)
        assert not isinstance(follower, (ProjectAirSimDrone, ProjectAirSimRover))
        assert follower._session is session
    finally:
        drone.close()
        rover.close()


def test_the_bare_bundle_built_from_live_objects_has_no_missing_slot():
    """The bundle the assembly builds per arm, made from the real classes."""
    drone, rover, _session = _live_parts()
    try:
        for vehicle in (drone, rover):
            ports = SimulatorPorts(
                vehicle=vehicle, observation=vehicle, path_follow=vehicle
            )
            assert ports.missing(PIPELINE_REQUIREMENTS["bare"]) == ()
            assert ports.missing(PIPELINE_REQUIREMENTS["staged"]) == ()
    finally:
        drone.close()
        rover.close()
