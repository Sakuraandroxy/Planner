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

from planner.application.mission_assembly import MissionResources
from planner.config import load_config
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
    from planner.simulators.airsim.session import AirSimMissionSession

    config = load_config("config/base.yaml")
    family = config.families.require("airsim")
    resources = MissionResources(
        detector=object(), planner=object(), geometry=object(),
        trajectory=None, view_cache=None, frame_sink=None,
    )
    return AirSimMissionSession(
        config,
        family,
        detector=resources.detector,
        planner=resources.planner,
        geometry=resources.geometry,
        log=lambda message, level="INFO": None,
        trajectory=resources.trajectory,
    )


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
    assert session.ports.capabilities() == airsim_runtime.describe().capabilities()


def test_free_navigation_is_not_built_for_ue4():
    """``ENABLED_TASKS`` decides what the family may be offered, not the config."""
    session = _session()
    kinds = {type(workflow).__name__ for workflow in session._arm.workflows}
    assert "FreeNavigationWorkflow" not in kinds, kinds
    assert {"BasicMotionWorkflow", "VisualGoalNavigationWorkflow"} <= kinds, kinds


def test_ue4_refuses_a_vehicle_it_does_not_drive():
    from planner.domain.server import TaskSubmission

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
    from planner.domain.pose import WorldPose

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
