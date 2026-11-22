"""Dual-vehicle session routing without a live ProjectAirSim server."""

import inspect
import threading
from types import SimpleNamespace

import pytest
from projectairsim.types import WeatherParameter

from planner.domain import MissionPlan, MissionStage, TaskKind
from planner.domain import WorldPose
from planner.domain import MissionResult
from planner.simulator import TaskSubmission
from planner.domain import VIEW_MODES
from planner.domain import WEATHER_FIELDS, default_weather
from planner.errors import ExecutionError, ProtocolError
from planner.simulators.projectairsim.session import (
    DEFAULT_CONTROL_PERIOD_S,
    HIDE_ALTITUDE_GAIN_M,
    HIDE_PIN_PERIOD_S,
    ProjectAirSimMissionSession,
    _VehicleArm,
)


def test_the_constructor_accepts_the_arguments_the_registry_passes():
    """``registry.build_session`` is the only caller; its kwargs must fit here.

    ``detector``, ``planner`` and ``geometry`` used to be required keyword
    arguments. The assembly moved to ``application/mission_assembly.py`` and
    stopped passing them, so every UE5 session build raised TypeError before
    touching the simulator. Required parameters are now the config and nothing
    else; the rest arrive through ``attach_arms``.
    """
    signature = inspect.signature(ProjectAirSimMissionSession.__init__)
    signature.bind(object(), object(), log=None, frame_sink=None, trajectory=None)
    required = [
        name
        for name, parameter in signature.parameters.items()
        if parameter.default is inspect.Parameter.empty
        and parameter.kind
        not in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD)
    ]
    assert required == ["self", "config"]


def test_build_session_forwards_the_sinks_the_console_reads(monkeypatch):
    """The session needs the frame and trajectory sinks; the assembly owns them.

    They reach the session as constructor arguments, like before the assembly was
    moved out. Dropping one is invisible until a task runs: the remaining-path
    panel, or the initial frame, simply stops updating.
    """
    from planner.config import default_config_path, load_config
    from planner.simulators.projectairsim import runtime as simulator_runtime
    from planner.simulators.projectairsim import session as session_module

    seen = {}

    class StubSession:
        def __init__(self, config, **kwargs):
            seen["config"] = config
            seen.update(kwargs)

    monkeypatch.setattr(
        session_module, "ProjectAirSimMissionSession", StubSession
    )
    config = load_config(default_config_path())
    entry = config.maps.entry("cityenviron")
    sinks = SimpleNamespace(frame_sink=object(), trajectory=object())

    simulator_runtime.build_session(
        config=config,
        map_entry=entry,
        family=config.families.require("projectairsim"),
        resources=sinks,
        log=lambda *args: None,
    )

    assert seen["frame_sink"] is sinks.frame_sink
    assert seen["trajectory"] is sinks.trajectory

class FakeApplication:
    def __init__(self, *, error=None):
        self.prepared = []
        self.executed = []
        self.error = error
        self.plan = MissionPlan(
            "mission_plan_v1",
            (MissionStage("stage_1", TaskKind.BASIC_MOTION, object()),),
        )

    def prepare(self, instruction):
        self.prepared.append(instruction)
        return self.plan

    def execute_plan(self, plan):
        self.executed.append(plan)
        if self.error is not None:
            raise self.error
        return MissionResult(True, ())

class FakeExecution:
    def __init__(self):
        self.checkpoints = []

    def bind_hold_checkpoint(self, checkpoint):
        self.checkpoints.append(checkpoint)

class FakeTrajectory:
    def __init__(self):
        self.started = []
        self.cleared = 0

    def start(self, instruction):
        self.started.append(instruction)

    def clear(self):
        self.cleared += 1

class FakeWorkflow:
    def __init__(self, name, order):
        self._name = name
        self._order = order

    def close(self):
        self._order.append(self._name)

class FakeModelCalls:
    def __init__(self, name, order):
        self._name = name
        self._order = order

    def close(self):
        self._order.append(self._name)

class FakeVehicle:
    def __init__(self, name, order=None):
        self.name = name
        self.calls = []
        self._order = order
        self.pose = WorldPose(10.0, 20.0, 1.0, 0.0)

    def current_pose(self):
        return self.pose

    def teleport(self, pose):
        self.calls.append(("teleport", pose))
        self.pose = pose

    def hover(self):
        self.calls.append("hover")

    def prepare(self, **kwargs):
        self.calls.append(("prepare", kwargs))

    def cancel(self):
        self.calls.append("cancel")

    def hold(self, **kwargs):
        self.calls.append(("hold", kwargs))

    def idle_tick(self):
        self.calls.append("idle_tick")

    def close(self):
        self.calls.append("close")
        if self._order is not None:
            self._order.append(f"{self.name}-close")

class FakeRover(FakeVehicle):
    def return_home(self):
        self.calls.append("return_home")
        return WorldPose(62.26, 0.0, 0.85, 12.5)

class FakeDrone(FakeVehicle):
    def __init__(self, name, order=None):
        super().__init__(name, order)
        self.landed = True
        self.land_ok = True

    def park(self):
        self.calls.append("park")

    def return_home(self):
        self.calls.append("return_home")
        return WorldPose(0.0, 0.0, 5.0, 90.0)

    def land(self):
        self.calls.append("land")
        if self.land_ok:
            self.landed = True
        if self._order is not None:
            self._order.append("land")
        return self.land_ok

class FakeConnection:
    def __init__(self, order=None):
        self.advances = []
        self._order = order
        self.world = object()
        self.reset_weather_calls = 0
        self.enable_calls = 0
        self.disable_calls = 0
        self.weather_params = []
        self.switch_calls = 0
        self.switch_ok = True
        self.now_ns = 0
        self.advance_hooks = []
        self.resume_clock_calls = 0
        self.free_running = True

    def resume_clock(self):
        self.resume_clock_calls += 1
        return self.free_running

    def add_advance_hook(self, hook):
        self.advance_hooks.append(hook)

    def remove_advance_hook(self, hook):
        if hook in self.advance_hooks:
            self.advance_hooks.remove(hook)

    def advance_for_sim_time(self, seconds):
        self.advances.append(seconds)
        self.now_ns += int(seconds * 1e9)
        for hook in tuple(self.advance_hooks):
            hook(self.now_ns)
        return self.now_ns

    def reset_weather_effects(self):
        self.reset_weather_calls += 1
        return True

    def enable_weather_effects(self):
        self.enable_calls += 1
        return True

    def disable_weather_effects(self):
        self.disable_calls += 1
        return True

    def set_weather_parameter(self, parameter, value):
        self.weather_params.append((parameter, value))
        return True

    def switch_streaming_view(self):
        self.switch_calls += 1
        return self.switch_ok

    def close(self):
        if self._order is not None:
            self._order.append("connection-close")

def make_session(*, active="rover", order=None, error=None):
    order = [] if order is None else order
    messages = []
    session = ProjectAirSimMissionSession.__new__(ProjectAirSimMissionSession)
    session._config = SimpleNamespace(
        server=SimpleNamespace(
            default_vehicle="rover", vehicles=("rover", "drone")
        ),
        projectairsim=SimpleNamespace(
            scene_file="scene_city_dual.jsonc",
            address="127.0.0.1",
            port_topics=8991,
            port_services=8992,
            rover_name="Rover1",
            drone_name="Drone1",
            front_camera="FrontCamera",
            chase_camera="Chase",
            uncapturable_views=("rover_chase",),
        ),
        simulator=SimpleNamespace(display=":104"),
    )
    session._trajectory = FakeTrajectory()
    session._log = lambda message, *args: messages.append(message)
    session._frame_sink = None
    session._publish_frames = False
    session._connection = FakeConnection(order)
    session._rover = FakeRover("rover", order)
    session._drone = FakeDrone("drone", order)
    session._active_vehicle = active
    session._hidden_poses = {}
    session._hidden_teleports = {}
    session._last_pin_ns = None
    session._pin_failures = set()
    session._pin_hook = session._pin_hidden
    session._connection.add_advance_hook(session._pin_hook)
    session._view_positions = VIEW_MODES
    # The runtime seeds this from projectairsim.view_cycle_start.
    session._view_index = VIEW_MODES.index("drone_front")
    session._view_mode = "drone_front"
    session._view_verified = False
    session._weather = None
    applications = {
        "rover": FakeApplication(error=error),
        "drone": FakeApplication(error=error),
    }
    session._arms = {
        "rover": _VehicleArm(
            "rover",
            session._rover,
            False,
            FakeExecution(),
            applications["rover"],
            FakeModelCalls("rover-model", order),
            (FakeWorkflow("rover-workflow", order),),
        ),
        "drone": _VehicleArm(
            "drone",
            session._drone,
            True,
            FakeExecution(),
            applications["drone"],
            FakeModelCalls("drone-model", order),
            (FakeWorkflow("drone-workflow", order),),
        ),
    }
    session.applications = applications
    session.messages = messages
    return session

def test_run_prepares_and_settles_the_rover():
    session = make_session()
    application = session.applications["rover"]
    result = session.run(TaskSubmission("task_1", "drive ahead", "rover"))
    assert result.success is True
    assert application.prepared == ["drive ahead"]
    assert application.executed == [application.plan]
    assert session._trajectory.started == ["drive ahead"]
    assert session._trajectory.cleared == 1
    # prepare() brakes, then finally settles with the same brake.
    assert session._rover.calls == ["hover", "hover"]
    assert session._drone.calls == []
    assert session._arms["rover"].execution.checkpoints == [None, None]
    assert any(
        "rover mission plan: stage_1:basic_motion" in message
        for message in session.messages
    )

def test_run_takes_off_for_the_drone_and_parks_the_rover():
    session = make_session()
    application = session.applications["drone"]
    result = session.run(TaskSubmission("task_1", "fly ahead", "drone"))
    assert result.success is True
    assert session._active_vehicle == "drone"
    assert session._rover.calls[0] == "hover"
    assert session._rover.calls[1][0] == "teleport"
    assert session._hidden_poses["rover"] == WorldPose(10.0, 20.0, 1.0, 0.0)
    assert ("prepare", {"takeoff_if_landed": True}) in session._drone.calls
    assert ("hold", {"require_airborne": False}) in session._drone.calls
    assert application.prepared == ["fly ahead"]
    assert any(
        "drone mission plan: stage_1:basic_motion" in message
        for message in session.messages
    )

def test_run_stops_the_drone_before_switching_to_the_rover():
    session = make_session(active="drone")
    result = session.run(TaskSubmission("task_1", "drive ahead", "rover"))
    assert result.success is True
    assert session._active_vehicle == "rover"
    assert "park" in session._drone.calls
    assert "land" not in session._drone.calls
    assert session._hidden_poses["drone"] == WorldPose(10.0, 20.0, 1.0, 0.0)
    assert session._rover.calls == ["hover", "hover"]

def test_run_rejects_an_unknown_vehicle():
    session = make_session()
    with pytest.raises(ProtocolError, match="unknown vehicle"):
        session.run(TaskSubmission("task_1", "fly", "boat"))
    assert session._rover.calls == []

def test_run_rejects_a_session_without_a_task_parser():
    session = make_session()
    session._arms["rover"].application = None
    with pytest.raises(ProtocolError, match="task parser"):
        session.run(TaskSubmission("task_1", "drive", "rover"))
    assert session._trajectory.started == []

def test_run_maps_a_stop_to_a_cancelled_result():
    session = make_session(error=KeyboardInterrupt())
    result = session.run(TaskSubmission("task_1", "drive", "rover"))
    assert result.success is False
    assert result.error == "cancelled: task was cancelled"
    assert session._trajectory.cleared == 1
    assert session._rover.calls[-1] == "hover"

def test_run_binds_the_stop_event_to_the_hold_checkpoint():
    session = make_session()
    stop_event = threading.Event()
    session.run(
        TaskSubmission("task_1", "drive", "rover"), stop_event=stop_event
    )
    checkpoint = session._arms["rover"].execution.checkpoints[0]
    assert callable(checkpoint)
    checkpoint()
    stop_event.set()
    with pytest.raises(KeyboardInterrupt):
        checkpoint()

def test_idle_tick_follows_the_active_vehicle():
    session = make_session(active="rover")
    session.idle_tick()
    assert session._rover.calls == ["idle_tick"]
    assert session._drone.calls == []

    session._active_vehicle = "drone"
    session.idle_tick()
    assert session._drone.calls == ["idle_tick"]

    session._drone = None
    session._rover = None
    session.idle_tick()
    assert session._connection.advances == [DEFAULT_CONTROL_PERIOD_S]

def test_return_home_routes_to_the_active_vehicle():
    session = make_session(active="rover")
    assert session.return_home() == "rover"
    assert session._rover.calls == ["return_home"]
    assert session._drone.calls == []
    assert any(
        "Rover returned to start position" in message
        for message in session.messages
    )

    session = make_session(active="drone")
    assert session.return_home() == "drone"
    assert session._drone.calls == ["return_home"]
    assert session._rover.calls == []
    assert any(
        "Drone returned to start position" in message
        for message in session.messages
    )

    session = make_session(active="rover")
    assert session.return_home("drone") == "drone"
    assert session._drone.calls[0] == "return_home"
    # Homing the inactive vehicle must not bring it back into view.
    assert session._drone.calls[1][0] == "teleport"
    assert "drone" in session._hidden_poses
    assert session._rover.calls == []

    with pytest.raises(ProtocolError, match="unknown vehicle"):
        session.return_home("boat")

def test_activate_vehicle_parks_the_rover_and_homes_the_drone():
    session = make_session(active="rover")
    assert session.active_vehicle == "rover"

    assert session.activate_vehicle("drone") == "drone"
    assert session.active_vehicle == "drone"
    assert session._rover.calls[0] == "hover"
    assert session._rover.calls[1][0] == "teleport"
    assert session._hidden_poses["rover"] == WorldPose(10.0, 20.0, 1.0, 0.0)
    assert session._drone.calls == ["return_home"]
    # The view walks from the declared start (drone_front) to the drone's chase
    # camera: one step forward.
    assert session._connection.switch_calls == 1
    assert session.view_mode == "drone_chase"
    assert any(
        "Vehicle activated: drone (parked rover" in message
        for message in session.messages
    )

def test_activate_vehicle_drops_the_parking_pin_before_homing_the_target():
    session = make_session(active="rover")
    parked = WorldPose(3.0, 4.0, 5.0, 0.0)
    hidden = WorldPose(3.0, 4.0, 1005.0, 0.0)
    session._hidden_poses["drone"] = parked
    session._hidden_teleports["drone"] = hidden
    session._drone.pose = hidden

    assert session.activate_vehicle("drone") == "drone"
    # One move, not two: the home teleport is the only thing sent, so the
    # altitude pin cannot drag the vehicle back to 1000 m.
    assert session._drone.calls == ["return_home"]
    assert "drone" not in session._hidden_poses
    assert "drone" not in session._hidden_teleports

def test_activate_vehicle_stops_an_airborne_drone_without_landing_it():
    # A native Land only replies at touchdown. It once outran the RPC timeout,
    # ProjectAirSim's client closed the connection, and the console got a scene
    # reload onto the default vehicle. So a switch must not ask for a landing,
    # and land_ok=False below must not be able to fail one.
    session = make_session(active="drone")
    session._view_index = 2
    session._view_mode = "drone_front"
    session._drone.land_ok = False

    assert session.activate_vehicle("rover") == "rover"
    assert session.active_vehicle == "rover"
    assert session._drone.calls[0] == "park"
    assert session._drone.calls[1] == (
        "teleport",
        WorldPose(10.0, 20.0, 1.0 - HIDE_ALTITUDE_GAIN_M, 0.0),
    )
    assert "land" not in session._drone.calls
    assert session._hidden_poses["drone"] == WorldPose(10.0, 20.0, 1.0, 0.0)
    assert session._rover.calls == ["return_home"]
    assert session._connection.switch_calls == 3
    assert session.view_mode == "rover_chase"

def test_activate_vehicle_can_repeat_the_active_vehicle():
    session = make_session(active="drone")
    session._view_index = 2
    session._view_mode = "drone_front"

    assert session.activate_vehicle("drone") == "drone"
    assert session.active_vehicle == "drone"
    assert session._rover.calls[0] == "hover"
    assert session._rover.calls[1][0] == "teleport"
    assert session._hidden_poses["rover"] == WorldPose(10.0, 20.0, 1.0, 0.0)
    assert session._drone.calls == ["return_home"]
    assert session._connection.switch_calls == 1
    assert session.view_mode == "drone_chase"

def test_activate_vehicle_rejects_an_unknown_vehicle():
    session = make_session()
    with pytest.raises(ProtocolError, match="unknown vehicle"):
        session.activate_vehicle("boat")
    assert session._rover.calls == []
    assert session._drone.calls == []

def test_hide_lifts_the_vehicle_and_unhide_restores_it():
    session = make_session(active="rover")
    parked = session._drone.pose

    session._hide("drone")
    hidden = WorldPose(
        parked.x,
        parked.y,
        parked.z - HIDE_ALTITUDE_GAIN_M,
        parked.yaw_deg,
    )
    assert session._drone.pose == hidden
    assert session._hidden_poses["drone"] == parked
    assert any("moved out of view" in message for message in session.messages)

    session._hide("drone")
    assert session._drone.calls == [("teleport", hidden)]

    session._unhide("drone")
    assert session._drone.pose == parked
    assert "drone" not in session._hidden_poses
    assert "drone" not in session._hidden_teleports
    assert any(
        "restored to its previous position" in message
        for message in session.messages
    )

def test_hidden_vehicle_is_pinned_at_altitude_while_the_world_steps():
    session = make_session(active="rover")
    session._hide("drone")
    hidden = session._drone.pose

    # Simulate gravity pulling the hidden drone down between clock advances.
    session._drone.pose = WorldPose(
        hidden.x, hidden.y, hidden.z + 40.0, hidden.yaw_deg
    )
    session._connection.advance_for_sim_time(HIDE_PIN_PERIOD_S)
    assert session._drone.pose == hidden

    # The pin is throttled: a short advance leaves the actor alone.
    session._drone.pose = WorldPose(
        hidden.x, hidden.y, hidden.z + 2.0, hidden.yaw_deg
    )
    session._connection.advance_for_sim_time(HIDE_PIN_PERIOD_S / 4.0)
    assert session._drone.pose.z == hidden.z + 2.0

    # After unhiding the hook stops touching the vehicle.
    session._unhide("drone")
    moved = WorldPose(1.0, 2.0, 3.0, 0.0)
    session._drone.pose = moved
    session._connection.advance_for_sim_time(HIDE_PIN_PERIOD_S)
    assert session._drone.pose == moved

def test_pin_failure_is_logged_once_and_recovered():
    session = make_session(active="rover")
    session._hide("drone")
    hidden = session._drone.pose
    original = session._drone.teleport
    failures = []

    def flaky(pose):
        failures.append(pose)
        raise RuntimeError("SetPose refused")

    session._drone.teleport = flaky
    session._connection.advance_for_sim_time(HIDE_PIN_PERIOD_S)
    session._connection.advance_for_sim_time(HIDE_PIN_PERIOD_S)
    assert len(failures) == 2
    assert sum(
        "could not keep the hidden drone at altitude" in message
        for message in session.messages
    ) == 1

    session._drone.teleport = original
    session._connection.advance_for_sim_time(HIDE_PIN_PERIOD_S)
    assert session._drone.pose == hidden
    assert "drone" not in session._pin_failures

def test_startup_hide_parks_the_inactive_vehicle_from_its_spawn():
    session = make_session(active="rover")
    spawn = session._drone.pose

    session._hide_inactive_at_startup()

    assert session._hidden_poses["drone"] == spawn
    assert session._hidden_teleports["drone"] == WorldPose(
        spawn.x, spawn.y, spawn.z - HIDE_ALTITUDE_GAIN_M, spawn.yaw_deg
    )
    assert session._drone.pose == session._hidden_teleports["drone"]

def test_activate_vehicle_fails_when_the_other_vehicle_cannot_be_hidden():
    session = make_session(active="rover")

    def refuse(pose):
        raise RuntimeError("SetPose refused")

    session._rover.teleport = refuse

    with pytest.raises(ExecutionError, match="could not hide the rover"):
        session.activate_vehicle("drone")
    assert session.active_vehicle == "rover"
    assert session._drone.calls == []
    assert session._connection.switch_calls == 0

def test_run_unhides_a_hidden_target_vehicle():
    session = make_session(active="rover")
    parked = WorldPose(0.0, 0.0, 5.0, 90.0)
    session._hidden_poses["drone"] = parked
    session._drone.pose = WorldPose(-300.0, -300.0, 5.0, 90.0)

    result = session.run(TaskSubmission("task_1", "fly ahead", "drone"))
    assert result.success is True
    assert ("teleport", parked) in session._drone.calls
    assert "drone" not in session._hidden_poses
    assert session.active_vehicle == "drone"

def test_return_home_clears_a_stale_hidden_record():
    session = make_session(active="rover")
    session._hidden_poses["rover"] = WorldPose(1.0, 2.0, 3.0, 0.0)
    session.return_home("rover")
    assert "rover" not in session._hidden_poses

def test_startup_hiding_failure_is_only_a_warning():
    session = make_session(active="rover")

    def refuse(pose):
        raise RuntimeError("SetPose refused")

    session._drone.teleport = refuse
    session._hide_inactive_at_startup()

    assert "drone" not in session._hidden_poses
    assert any(
        "could not move the drone out of view" in message
        for message in session.messages
    )

def test_close_closes_workflows_before_vehicles_and_the_connection():
    order = []
    session = make_session(order=order)
    session._drone.landed = False
    session.close()
    assert order == [
        "rover-workflow",
        "rover-model",
        "drone-workflow",
        "drone-model",
        "land",
        "drone-close",
        "rover-close",
        "connection-close",
    ]
    assert session._arms == {}
    assert session._drone is None
    assert session._rover is None
    assert session._connection is None
    with pytest.raises(RuntimeError):
        session.return_home()

def test_close_skips_drone_landing_when_the_scene_is_replaced():
    order = []
    session = make_session(order=order)
    session._drone.landed = False
    session.close(skip_landing=True)
    assert "land" not in order
    assert any(
        "Skipping drone landing" in message for message in session.messages
    )
    assert session._drone is None
    assert session._connection is None

class UnreadableLandedDrone:
    """A drone whose state can no longer be read from a torn-down simulator."""

    def __init__(self, order):
        self._order = order

    @property
    def landed(self):
        raise RuntimeError("Object closed")

    def close(self):
        self._order.append("drone-close")

def test_close_releases_the_connection_when_the_landing_state_raises():
    order = []
    session = make_session(order=order)
    session._drone = UnreadableLandedDrone(order)

    session.close(skip_landing=True)

    # The process-wide single-consumer guard lives in connection.close(); a
    # failed landing-state read must not skip it.
    assert "connection-close" in order
    assert session._connection is None
    assert any(
        "could not confirm the drone landing state" in message
        for message in session.messages
    )

def test_weather_catalog_is_static_and_complete():
    session = make_session()
    catalog = session.weather_catalog()
    assert [field.id for field in catalog.fields] == list(WEATHER_FIELDS)
    assert session.weather is None
    assert session._connection.reset_weather_calls == 0

def test_set_weather_enables_effects_and_maps_each_nonzero_field():
    session = make_session()
    values = default_weather()
    values.update({"rain": 0.5, "road_wetness": 0.75, "fog": 0.25})
    applied = session.set_weather(values)

    assert applied == values
    assert session.weather == values
    connection = session._connection
    assert connection.reset_weather_calls == 1
    assert connection.enable_calls == 1
    assert connection.disable_calls == 0
    assert dict(connection.weather_params) == {
        int(WeatherParameter.RAIN): 0.5,
        int(WeatherParameter.ROAD_WETNESS): 0.75,
        int(WeatherParameter.FOG): 0.25,
    }
    assert any("Weather applied:" in message for message in session.messages)

def test_set_weather_all_zero_disables_effects():
    session = make_session()
    values = default_weather()
    session.set_weather(values)
    connection = session._connection
    assert connection.reset_weather_calls == 1
    assert connection.enable_calls == 0
    assert connection.disable_calls == 1
    assert connection.weather_params == []
    assert any(
        "Weather cleared;" in message for message in session.messages
    )

def test_set_weather_requires_all_eight_fields():
    session = make_session()
    with pytest.raises(ValueError, match="missing weather fields"):
        session.set_weather({"rain": 0.5})
    assert session._connection.weather_params == []

def test_set_view_mode_counts_switches_from_the_declared_start():
    """The ring position is bookkeeping, not a question to ask the engine.

    ``switch_streaming_view`` walks the cameras forward and reports nothing, so
    the position is tracked from ``projectairsim.view_cycle_start``: the camera
    the engine is on right after a scene load.
    """
    session = make_session()
    assert session.view_mode == "drone_front"
    assert session.set_view_mode("drone_front") == "drone_front"
    assert session._connection.switch_calls == 0

    session._connection.switch_calls = 0
    assert session.set_view_mode("rover_chase") == "rover_chase"
    # drone_front is position 2, rover_chase is 1: three steps forward.
    assert session._connection.switch_calls == 3

    session._connection.switch_calls = 0
    assert session.set_view_mode("rover_front") == "rover_front"
    assert session._connection.switch_calls == 3

    # Naming the start is what makes a wrong view_cycle_start diagnosable: the
    # buttons would all be off by the same number of steps.
    assert any(
        "from drone_front" in message for message in session.messages
    ), session.messages

def test_set_view_mode_reports_a_simulator_refusal():
    session = make_session()
    session._connection.switch_ok = False
    with pytest.raises(ProtocolError, match="refused to switch"):
        session.set_view_mode("rover_chase")
    # Nothing was applied, so the console is still on the declared start.
    assert session.view_mode == "drone_front"

def test_set_view_mode_rejects_an_unknown_mode():
    session = make_session()
    with pytest.raises(ValueError, match="unknown view mode"):
        session.set_view_mode("moon")
    assert session._connection.switch_calls == 0

class FakeSight:
    """Stand in for reading the camera off the desktop."""

    def __init__(self, observed):
        self._observed = list(observed)
        self.calls = 0

    def __call__(self):
        value = self._observed[min(self.calls, len(self._observed) - 1)]
        self.calls += 1
        return value

