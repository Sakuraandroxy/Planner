"""Dual-vehicle session routing without a live ProjectAirSim server."""

import threading
from types import SimpleNamespace

import pytest

from planner.adapters.runtime.projectairsim_session import (
    DEFAULT_CONTROL_PERIOD_S,
    HIDE_ALTITUDE_GAIN_M,
    HIDE_PIN_PERIOD_S,
    ProjectAirSimMissionSession,
    _VehicleArm,
)
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.pose import WorldPose
from planner.domain.result import MissionResult
from planner.domain.server import TaskSubmission
from planner.domain.view import INITIAL_STREAM_INDEX, VIEW_MODES
from planner.domain.weather import WEATHER_FIELDS, default_weather
from planner.errors import ExecutionError, ProtocolError
from projectairsim.types import WeatherParameter


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
    session._view_cache = None
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
    session._view_index = INITIAL_STREAM_INDEX
    session._view_mode = VIEW_MODES[INITIAL_STREAM_INDEX]
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


def test_run_lands_the_drone_before_switching_to_the_rover():
    session = make_session(active="drone")
    session._drone.landed = False
    result = session.run(TaskSubmission("task_1", "drive ahead", "rover"))
    assert result.success is True
    assert session._active_vehicle == "rover"
    assert "cancel" in session._drone.calls
    assert "land" in session._drone.calls
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
    assert session._connection.switch_calls == 3
    assert session.view_mode == "drone_chase"
    assert any(
        "Vehicle activated: drone (parked rover" in message
        for message in session.messages
    )


def test_activate_vehicle_restores_a_hidden_target_before_homing_it():
    session = make_session(active="rover")
    parked = WorldPose(3.0, 4.0, 5.0, 0.0)
    session._hidden_poses["drone"] = parked
    session._hidden_teleports["drone"] = WorldPose(3.0, 4.0, 1005.0, 0.0)

    assert session.activate_vehicle("drone") == "drone"
    # The target must leave the hidden record first: the altitude pin would
    # otherwise fight the home teleport on every clock advance.
    assert session._drone.calls[0] == ("teleport", parked)
    assert session._drone.calls[1] == "return_home"
    assert "drone" not in session._hidden_poses
    assert "drone" not in session._hidden_teleports


def test_activate_vehicle_lands_the_drone_before_activating_the_rover():
    session = make_session(active="drone")
    session._view_index = 2
    session._view_mode = "drone_front"
    session._drone.landed = False

    assert session.activate_vehicle("rover") == "rover"
    assert session.active_vehicle == "rover"
    assert session._drone.calls[:2] == ["cancel", "land"]
    assert session._drone.calls[2][0] == "teleport"
    assert session._hidden_poses["drone"] == WorldPose(10.0, 20.0, 1.0, 0.0)
    assert session._rover.calls == ["return_home"]
    assert session._connection.switch_calls == 3
    assert session.view_mode == "rover_chase"


def test_activate_vehicle_fails_when_the_drone_landing_is_unconfirmed():
    session = make_session(active="drone")
    session._view_index = 2
    session._view_mode = "drone_front"
    session._drone.landed = False
    session._drone.land_ok = False

    with pytest.raises(ExecutionError, match="could not park the drone"):
        session.activate_vehicle("rover")
    assert session.active_vehicle == "drone"
    assert session._rover.calls == []
    assert session._connection.switch_calls == 0
    assert any(
        "landing was not confirmed" in message
        for message in session.messages
    )


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


def test_set_view_mode_cycles_forward_to_the_target():
    session = make_session()
    assert session.view_mode == "rover_front"
    assert session.set_view_mode("drone_front") == "drone_front"
    assert session._connection.switch_calls == 2
    assert session.view_mode == "drone_front"

    session._connection.switch_calls = 0
    assert session.set_view_mode("rover_chase") == "rover_chase"
    assert session._connection.switch_calls == 3

    session._connection.switch_calls = 0
    assert session.set_view_mode("rover_chase") == "rover_chase"
    assert session._connection.switch_calls == 0


def test_set_view_mode_reports_a_simulator_refusal():
    session = make_session()
    session._connection.switch_ok = False
    with pytest.raises(ProtocolError, match="refused to switch"):
        session.set_view_mode("rover_chase")
    assert session.view_mode == "rover_front"


def test_set_view_mode_rejects_an_unknown_mode():
    session = make_session()
    with pytest.raises(ValueError, match="unknown view mode"):
        session.set_view_mode("moon")
    assert session._connection.switch_calls == 0


def test_set_view_mode_uses_the_calibrated_cycle_order():
    session = make_session()
    # The live server may cycle in a different order than the built-in list.
    session._view_positions = (
        "rover_chase",
        "rover_front",
        "drone_front",
        "drone_chase",
    )
    session._view_index = 0
    session._view_mode = "rover_chase"

    assert session.set_view_mode("drone_front") == "drone_front"
    assert session._connection.switch_calls == 2
    assert session.view_mode == "drone_front"

    session._connection.switch_calls = 0
    assert session.set_view_mode("rover_chase") == "rover_chase"
    assert session._connection.switch_calls == 2
    assert session.view_mode == "rover_chase"


def test_calibrate_views_stores_the_measured_order(monkeypatch):
    from planner.adapters.runtime import projectairsim_session as module

    session = make_session()
    session._connection.world = SimpleNamespace(
        parent_topic="/Sim/SceneTest"
    )
    measured = (
        "rover_chase",
        "rover_front",
        "drone_front",
        "drone_chase",
    )

    class Fetcher:
        def __init__(self):
            self.closed = 0

        def __call__(self, robot, camera):
            return None

        def close(self):
            self.closed += 1

    fetcher = Fetcher()
    monkeypatch.setattr(
        module, "service_frame_fetcher", lambda **kwargs: fetcher
    )
    monkeypatch.setattr(
        module,
        "calibrate_streaming_views",
        lambda **kwargs: measured,
    )
    monkeypatch.setattr(module, "xpra_screenshot", lambda display: object())

    session._calibrate_views()

    assert session._view_positions == measured
    # Calibration leaves the server at the measured anchor (rover_chase),
    # which is also the default vehicle's chase view: no extra switch.
    assert session._view_index == 0
    assert session._view_mode == "rover_chase"
    assert session._connection.switch_calls == 0
    assert fetcher.closed == 1
    assert any(
        "Streaming views calibrated" in message
        for message in session.messages
    )


def test_calibrate_views_keeps_the_builtin_order_when_measurement_fails(
    monkeypatch,
):
    from planner.adapters.runtime import projectairsim_session as module

    session = make_session()
    session._connection.world = SimpleNamespace(
        parent_topic="/Sim/SceneTest"
    )
    monkeypatch.setattr(
        module, "service_frame_fetcher", lambda **kwargs: (lambda robot, camera: None)
    )
    monkeypatch.setattr(
        module, "calibrate_streaming_views", lambda **kwargs: None
    )
    monkeypatch.setattr(module, "xpra_screenshot", lambda display: object())

    session._calibrate_views()

    assert session._view_positions == VIEW_MODES
    # The fallback still parks the console on the default vehicle's chase
    # (tail) camera, matching what the initial console state promises.
    assert session._view_mode == "rover_chase"
    assert any(
        "falling back to the built-in view order" in message
        for message in session.messages
    )


def test_calibrate_views_reuses_a_cached_order_after_the_anchor_check(
    monkeypatch,
):
    from planner.adapters.runtime import projectairsim_session as module

    session = make_session()
    session._connection.world = SimpleNamespace(
        parent_topic="/Sim/SceneTest"
    )
    cached = (
        "rover_front",
        "drone_front",
        "rover_chase",
        "drone_chase",
    )
    session._view_cache = {"scene_city_dual.jsonc": cached}
    calls = {}

    class Fetcher:
        def __call__(self, robot, camera):
            return None

        def close(self):
            pass

    monkeypatch.setattr(
        module, "service_frame_fetcher", lambda **kwargs: Fetcher()
    )
    monkeypatch.setattr(module, "xpra_screenshot", lambda display: object())

    def verify(**kwargs):
        calls["verify"] = kwargs
        return "drone_front"

    monkeypatch.setattr(module, "verify_streaming_views", verify)
    monkeypatch.setattr(
        module,
        "calibrate_streaming_views",
        lambda **kwargs: (_ for _ in ()).throw(
            AssertionError("full calibration should not run")
        ),
    )

    session._calibrate_views()

    assert session._view_positions == cached
    # The cached order is anchored at drone_front, then the default vehicle's
    # chase view is applied (index 2).
    assert session._view_index == 2
    assert session._view_mode == "rover_chase"
    assert calls["verify"]["uncapturable"] == ("rover_chase",)
    assert any("reused from cache" in message for message in session.messages)


def test_calibrate_views_caches_a_fresh_measurement(monkeypatch):
    from planner.adapters.runtime import projectairsim_session as module

    session = make_session()
    session._connection.world = SimpleNamespace(
        parent_topic="/Sim/SceneTest"
    )
    session._view_cache = {}
    measured = (
        "rover_chase",
        "rover_front",
        "drone_front",
        "drone_chase",
    )
    monkeypatch.setattr(
        module, "service_frame_fetcher", lambda **kwargs: (lambda r, c: None)
    )
    monkeypatch.setattr(
        module, "calibrate_streaming_views", lambda **kwargs: measured
    )
    monkeypatch.setattr(module, "xpra_screenshot", lambda display: object())

    session._calibrate_views()

    assert session._view_cache == {"scene_city_dual.jsonc": measured}


def test_calibrate_views_recalibrates_when_the_cache_does_not_verify(
    monkeypatch,
):
    from planner.adapters.runtime import projectairsim_session as module

    session = make_session()
    session._connection.world = SimpleNamespace(
        parent_topic="/Sim/SceneTest"
    )
    stale = (
        "rover_front",
        "drone_front",
        "rover_chase",
        "drone_chase",
    )
    session._view_cache = {"scene_city_dual.jsonc": stale}
    measured = (
        "drone_front",
        "rover_chase",
        "drone_chase",
        "rover_front",
    )
    monkeypatch.setattr(
        module, "service_frame_fetcher", lambda **kwargs: (lambda r, c: None)
    )
    monkeypatch.setattr(
        module, "verify_streaming_views", lambda **kwargs: None
    )
    monkeypatch.setattr(
        module, "calibrate_streaming_views", lambda **kwargs: measured
    )
    monkeypatch.setattr(module, "xpra_screenshot", lambda display: object())

    session._calibrate_views()

    assert session._view_positions == measured
    assert session._view_cache == {"scene_city_dual.jsonc": measured}
    assert any(
        "did not verify" in message for message in session.messages
    )


def test_calibrate_views_skips_without_a_scene_topic():
    session = make_session()
    session._connection.world = SimpleNamespace()

    session._calibrate_views()

    assert session._view_positions == VIEW_MODES
    assert any(
        "no scene topic" in message for message in session.messages
    )

