"""The UE4 session must assemble into the ports its package advertises.

Nothing here touches a simulator: ``airsim`` is faked, and the point is the wiring
around it -- which slots get filled, which tasks the family is allowed to be offered,
and how the planner's canonical weather names reach AirSim's own spelling.

The ``describe()`` versus ``build_ports()`` assertion is the one
``planner/simulators/airsim/runtime.py`` promises in its docstring but that nobody
wrote until this file: the console is told one thing before a simulator runs and the
mission must not be handed a different bundle once it does.
"""

import sys
import types

import pytest

from planner.assembly import MissionResources
from planner.assembly import requirements
from planner.config import default_config_path, load_config
from planner.domain import WEATHER_FIELDS
from planner.errors import ProtocolError
from planner.simulators.airsim import runtime as airsim_runtime


class _Future:
    def get(self):
        return True

    def attach_callback(self, callback):
        pass


class _Client:
    def __init__(self, **kwargs):
        self.calls: list[tuple] = []

    def _record(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))
        return True

    def __getattr__(self, name):
        def proxy(*args, **kwargs):
            if name.endswith("Async"):
                return _Future()
            self._record(name, *args, **kwargs)
            return True

        return proxy


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    module = types.ModuleType("airsim")
    module.MultirotorClient = _Client
    monkeypatch.setitem(sys.modules, "airsim", module)


def _session():
    """Build the session the way the registry does: ports first, then wiring.

    The session no longer receives the planner stack. It hands over
    ``mission_inputs()`` and the shared assembly builds the arm from the
    service-side resources -- which is the arrangement this test exists to keep
    honest, because the UE4 mission child once skipped one of those resources
    and the console quietly lost its trajectory panel.
    """
    from planner.assembly import assemble_missions
    from planner.runtime import TrajectoryHub
    from planner.simulators.airsim.session import AirSimMissionSession

    config = load_config(default_config_path())
    family = config.families.require("airsim")
    quiet = lambda message, level="INFO": None
    resources = MissionResources(
        detector=object(), planner=object(), geometry=object(),
        trajectory=TrajectoryHub(), frame_sink=None,
    )
    session = AirSimMissionSession(config, family, log=quiet)
    session.attach_arms(
        assemble_missions(
            config, inputs=session.mission_inputs(), resources=resources, log=quiet
        )
    )
    return session


def test_the_session_fills_exactly_the_declared_slots():
    session = _session()
    declared = set(airsim_runtime.PORT_SLOTS)
    filled = {slot for slot in declared if session.ports.slot(slot) is not None}
    assert filled == declared, (
        f"describe() advertises {sorted(declared)} but the live session leaves "
        f"{sorted(declared - filled)} unimplemented"
    )


def test_the_live_bundle_agrees_with_the_advertised_capabilities():
    session = _session()
    assert requirements.capabilities_for(session.ports) == requirements.capabilities_for(
        airsim_runtime.describe()
    )


def test_free_navigation_is_built_for_ue4():
    """The streaming slot is filled, so cruising is offered without a whitelist (R18)."""
    session = _session()
    kinds = {type(workflow).__name__ for workflow in session._arm.workflows}
    assert {
        "BasicMotionWorkflow",
        "VisualGoalNavigationWorkflow",
        "FreeNavigationWorkflow",
    } <= kinds, kinds


def test_ue4_refuses_a_vehicle_it_does_not_drive():
    from planner.domain import TaskSubmission

    session = _session()
    with pytest.raises(ProtocolError, match="drone only"):
        session.run(TaskSubmission(task_id="t", instruction="fly east", vehicle="rover"))


def test_weather_reaches_air_sim_in_the_engine_spelling():
    session = _session()
    # ``set_weather`` takes a complete eight-field state, the same contract the HTTP
    # endpoint enforces; only ``road_wetness`` differs from the engine's spelling.
    fields = {
        "rain": 0.5, "road_wetness": 0.2, "snow": 0.0, "road_snow": 0.0,
        "maple_leaf": 0.0, "road_leaf": 0.0, "dust": 0.0, "fog": 0.1,
    }
    applied = session.set_weather(fields)

    client = session.connection.client
    sent = {args[:2] for name, args, _ in client.calls if name == "simSetWeatherParameter"}
    assert (0, 0.5) in sent and (1, 0.2) in sent and (7, 0.1) in sent, sent
    assert (1, 0.2) in sent, "road_wetness must be sent as the engine's roadwetness"
    assert len(sent) == 8, sent
    assert applied["road_wetness"] == 0.2, "the caller keeps the canonical name it sent"


def _weather(**overrides):
    """A complete eight-field state; ``set_weather`` always sends all of them."""
    values = {name: 0.0 for name in WEATHER_FIELDS}
    values.update(overrides)
    return values


def test_weather_switches_the_effects_on_before_it_sets_parameters():
    """AirSim ignores every weather parameter until ``simEnableWeather`` has run."""
    session = _session()
    session.set_weather(_weather(rain=0.5, fog=0.1))

    calls = session.connection.client.calls
    assert calls[0][:2] == ("simEnableWeather", (True,)), calls[0]
    assert "simSetWeatherParameter" in [name for name, _, _ in calls[1:]]


def test_a_clear_sky_switches_the_effects_back_off():
    session = _session()
    session.set_weather(_weather())

    calls = session.connection.client.calls
    assert calls[0][:2] == ("simEnableWeather", (False,)), calls[0]
    assert len(calls) == 9, "the eight zero parameters still go out"


def test_a_session_that_never_flew_releases_transports_without_landing():
    """A session that never left the ground has nothing to land.

    The ground record is taken when a task prepares the vehicle, so ``close`` before
    any flight must not issue a descent nobody asked for -- it only releases the
    transports.
    """
    session = _session()
    closed: list[str] = []
    calls: list[str] = []
    session.connection.close_transport = lambda: closed.append("control")
    session.connection.call_async_and_wait = lambda name, *a, **k: calls.append(name)
    session.connection.shutdown = lambda **kwargs: calls.append("shutdown")

    session.close()
    assert calls == [], calls
    assert closed == ["control"]


def test_close_lands_before_it_releases_the_transport():
    """The gap the first real UE4 task exposed.

    ``AirSimConnection.shutdown`` refuses to disarm without positive landed evidence
    and never lands by itself, so a task ending on a hover left the motors enabled
    while the stack was killed (measured: the drone was still at z=-0.83 when the
    engine died). The session owns the landing now: descend, confirm by height,
    then shut down, then release.
    """
    from planner.domain import WorldPose

    session = _session()
    session._ground_z = 0.68
    order: list[str] = []
    session.connection.call_async_and_wait = lambda name, *a, **k: (
        order.append("landAsync") if name == "landAsync" else None
    )
    session.connection.shutdown = lambda **kwargs: order.append("shutdown")
    session.connection.close_transport = lambda: order.append("close_transport")
    session.server.control.current_pose = lambda: WorldPose(0.0, 0.0, 0.68, 0.0)

    session.close()
    assert order == ["landAsync", "shutdown", "close_transport"], order


def test_close_without_landing_releases_every_transport():
    """``skip_landing`` is a map switch: the scene is about to be destroyed."""
    session = _session()
    closed = []
    session.connection.close_transport = lambda: closed.append("control")
    read = _Client()
    session._capture_connections.append(read)
    read.close_transport = lambda: closed.append("capture")

    session.close(skip_landing=True)
    assert closed == ["capture", "control"]
    session.close(skip_landing=True)          # idempotent: the worker may retry
    assert closed == ["capture", "control"]


def test_assembly_refuses_to_run_without_a_trajectory_sink():
    """A missing sink must be an error, not a quietly missing console panel.

    The UE4 mission child built its own resources and left ``trajectory`` out.
    The visual-goal workflow answered ``None`` by publishing nothing, so the
    console lost the remaining-path panel with no error anywhere -- and the
    capability list blamed the simulator instead.
    """
    from planner.assembly import MissionInputs, assemble_missions

    config = load_config(default_config_path())
    resources = MissionResources(
        detector=object(), planner=object(), geometry=object(),
        trajectory=None, frame_sink=None,
    )
    with pytest.raises(ProtocolError, match="trajectory sink"):
        assemble_missions(
            config,
            inputs=MissionInputs(vehicles={"drone": object()}),
            resources=resources,
            log=lambda message, level="INFO": None,
        )


def test_a_session_without_its_arm_is_refused_at_wiring_time():
    from planner.simulators.airsim.session import AirSimMissionSession

    config = load_config(default_config_path())
    family = config.families.require("airsim")
    session = AirSimMissionSession(
        config, family, log=lambda message, level="INFO": None
    )
    with pytest.raises(ProtocolError, match="no drone arm"):
        session.attach_arms({"rover": object()})


def test_moving_capture_does_not_share_the_control_transport():
    """Frame requests run on a worker thread; the control loop must not wait on them.

    ``simGetImages`` holds the connection's lock for 0.4-0.5 s on this build. When
    moving capture shared the control connection, sixteen of 66 s of control ticks
    in a planner-driven flight waited that long, the velocity command expired in
    the gap, and the aircraft braked and re-accelerated through every planning
    round -- visible as a rolling, pitching wobble that only UE4 showed, because
    ProjectAirSim's stepped clock does not advance while the loop is blocked.
    """
    session = _session()
    moving = session.mission_inputs().moving_source_factory(None)
    assert moving is not session.observation
    assert moving.connection is not session.connection
    assert moving.connection in session._capture_connections
