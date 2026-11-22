import queue
import time

import pytest

from planner.assembly.controller import ServerController
from planner.config import (
    MapCatalog,
    MapEntry,
    MapScene,
    ProfileInfo,
    ServerSettings,
    VehicleModelCatalog,
    VehicleModelGroup,
    VehicleModelOption,
)
from planner.domain.pose import WorldPose
from planner.domain.server import (
    BackendUnavailable,
    MapSelection,
    ServerState,
    WorkerEvent,
    WorkerEventKind,
)
from planner.domain.sliding_window import QueuedWaypoint
from planner.domain.trajectory_telemetry import MODE_CONTINUOUS
from planner.domain.weather import WEATHER_FIELDS
from planner.errors import SimulatorNotOwned
from planner.runtime.trajectory_hub import TrajectoryHub


def _settings(**overrides):
    values = dict(
        log_buffer_size=100,
        reconnect_interval_s=0.01,
        startup_timeout_s=1.0,
    )
    values.update(overrides)
    return ServerSettings(**values)


def _maps():
    return MapCatalog(
        default_map="city",
        maps=(
            MapEntry(
                id="city",
                label="城市",
                launch_map="cityenviron",
                simulator="projectairsim",
                scene=MapScene(
                    file="scene_city_dual.jsonc",
                    config_path="config/scenes/city",
                    vehicle_scenes=(
                        ("hexarotor", "scene_city_dual_hexarotor.jsonc"),
                    ),
                ),
            ),
            MapEntry(
                id="suburb",
                label="郊区",
                launch_map="neighborhood",
                simulator="projectairsim",
                scene=MapScene(
                    file="scene_neighborhood_dual.jsonc",
                    config_path="config/scenes/neighborhood",
                    vehicle_scenes=(
                        ("hexarotor", "scene_neighborhood_dual_hexarotor.jsonc"),
                    ),
                ),
            ),
        ),
    )


def _models():
    return VehicleModelCatalog(
        groups=(
            VehicleModelGroup(
                "rover",
                "offroad",
                (VehicleModelOption("offroad", "越野车"),),
            ),
            VehicleModelGroup(
                "drone",
                "quadrotor",
                (
                    VehicleModelOption("quadrotor", "四旋翼"),
                    VehicleModelOption("hexarotor", "六旋翼"),
                ),
            ),
        )
    )


class FakeWorker:
    def __init__(self):
        self.events = queue.Queue()
        self.tasks = []
        self.started = False
        self.stops = 0
        self.homes = []
        self.activations = []
        self.views = []
        self.weathers = []
        self.closed = False
        self.skip_landing = None

    def emit(self, kind, **kwargs):
        self.events.put(WorkerEvent(kind, **kwargs))

    def start(self):
        self.started = True
        self.emit(WorkerEventKind.READY)

    def submit(self, task):
        self.tasks.append(task)

    def stop(self):
        self.stops += 1

    def home(self, vehicle=None):
        self.homes.append(vehicle)

    def activate(self, vehicle):
        self.activations.append(vehicle)

    def set_view_mode(self, mode):
        self.views.append(mode)

    def set_weather(self, fields):
        self.weathers.append(dict(fields))

    def poll(self):
        try:
            return self.events.get_nowait()
        except queue.Empty:
            return None

    def shutdown(self, *, skip_landing=False):
        self.closed = True
        self.skip_landing = skip_landing
        self.emit(WorkerEventKind.CLOSED)

    def terminate(self):
        self.closed = True
        self.emit(WorkerEventKind.CLOSED)


class SilentWorker(FakeWorker):
    """A worker that never confirms CLOSED, emulating a hung session."""

    def shutdown(self, *, skip_landing=False):
        pass

    def terminate(self):
        pass


class FakeSupervisor:
    def __init__(
        self,
        *,
        fail_times=0,
        preflight_error=None,
        running=False,
        launch_fail_times=0,
    ):
        self.restarts = []
        self.launches = []
        self.stops = []
        self.preflight_calls = 0
        self.fail_times = fail_times
        self.preflight_error = preflight_error
        self.running_state = running
        self.launch_fail_times = launch_fail_times

    def running(self):
        return self.running_state

    def launch_command(self, entry):
        return f"scripts/launch {entry.launch_map}"

    def launch(self, entry):
        self.launches.append(entry.id)
        if self.launch_fail_times > 0:
            self.launch_fail_times -= 1
            raise RuntimeError("simulator launch boom")
        self.running_state = True

    def start(self, entry):
        if self.running_state:
            raise RuntimeError("a simulator stack is already running")
        self.launch(entry)

    def preflight(self):
        self.preflight_calls += 1
        if self.preflight_error is not None:
            raise self.preflight_error

    def stop(self):
        self.stops.append(True)
        self.running_state = False

    def restart(self, entry):
        self.restarts.append(entry.id)
        if self.fail_times > 0:
            self.fail_times -= 1
            raise RuntimeError("simulator restart boom")
        self.running_state = True


def _wait(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition was not met in time")


def _controller(workers, *, worker_class=FakeWorker, **overrides):
    selections = []
    supervisor = overrides.pop("simulator", None)
    restart_attempts = overrides.pop("restart_attempts", 2)
    shutdown_timeout = overrides.pop("worker_shutdown_timeout_s", 60.0)
    vehicle_models = overrides.pop("vehicle_models", None)
    autostart_session = overrides.pop("autostart_session", True)
    session_reset = overrides.pop("session_reset", None)
    profiles = overrides.pop("profiles", ())
    initial_profile = overrides.pop("initial_profile", "")

    def factory(selection):
        selections.append(selection)
        worker = worker_class()
        workers.append(worker)
        return worker

    controller = ServerController(
        factory,
        _settings(**overrides),
        maps=_maps(),
        simulator=supervisor,
        initial_map="city",
        vehicle_models=vehicle_models,
        restart_attempts=restart_attempts,
        worker_shutdown_timeout_s=shutdown_timeout,
        autostart_session=autostart_session,
        session_reset=session_reset,
        profiles=profiles,
        initial_profile=initial_profile,
    )
    controller._test_selections = selections
    return controller


def _ready_controller(workers, **kwargs):
    controller = _controller(workers, **kwargs)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    return controller


def test_submit_is_unavailable_before_connecting():
    controller = _controller([])
    with pytest.raises(BackendUnavailable) as error:
        controller.submit("fly")
    assert error.value.state is ServerState.CONNECTING


def test_ready_then_task_lifecycle():
    workers = []
    controller = _controller(workers)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    worker = workers[0]
    assert worker.started
    assert controller.status.map == "city"
    assert controller.status.maps == ("city", "suburb")

    task = controller.submit("  fly to the red car  ")
    _wait(lambda: worker.tasks)
    assert task.instruction == "fly to the red car"
    assert controller.state is ServerState.RUNNING
    assert controller.status.instruction == "fly to the red car"

    worker.emit(
        WorkerEventKind.FINISHED,
        task_id=task.task_id,
        success=True,
        message="reached: close enough",
    )
    _wait(lambda: controller.state is ServerState.IDLE)
    assert controller.status.instruction == ""
    assert controller.status.last_result == "reached: close enough"
    assert "Task task_1 ended" in "\n".join(
        item.message for item in controller.logs.read(0).items
    )

    controller.close()
    assert worker.closed


def test_starts_disconnected_without_creating_a_worker():
    workers = []
    supervisor = FakeSupervisor(running=False)
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()
    time.sleep(0.05)

    assert controller.state is ServerState.DISCONNECTED
    assert workers == []
    assert supervisor.launches == []
    assert supervisor.stops == []
    with pytest.raises(BackendUnavailable) as error:
        controller.submit("fly")
    assert error.value.state is ServerState.DISCONNECTED
    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "No scene is loaded" in messages
    controller.close()


def test_scene_start_launches_an_absent_stack_then_connects():
    workers = []
    supervisor = FakeSupervisor(running=False)
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()

    assert controller.request_world("city") is ServerState.CONNECTING
    _wait(lambda: controller.state is ServerState.IDLE)

    assert supervisor.launches == ["city"]
    assert supervisor.restarts == []
    assert supervisor.preflight_calls == 0
    assert controller._test_selections[0].scene_file == "scene_city_dual.jsonc"
    controller.close()


def test_start_stops_a_leftover_script_managed_stack():
    workers = []
    supervisor = FakeSupervisor(running=True)
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()

    assert supervisor.stops == [True]
    assert supervisor.running_state is False
    assert supervisor.launches == []
    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "Leftover simulator stopped" in messages
    controller.close()


def test_start_leaves_an_unowned_stack_alone():
    workers = []
    supervisor = FakeSupervisor(
        running=True, preflight_error=SimulatorNotOwned("no pidfiles")
    )
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()

    assert supervisor.stops == []
    assert supervisor.running_state is True
    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "leaving it alone" in messages
    controller.close()


# -- shutdown takes the stack with it -------------------------------------
def test_stop_world_closes_the_session_and_stops_the_stack():
    """The console needs a way to take a scene down without exiting."""
    workers = []
    supervisor = FakeSupervisor(running=False)
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()
    controller.request_world("city")
    _wait(lambda: controller.state is ServerState.IDLE)
    worker = workers[0]

    assert controller.stop_world() is ServerState.CONNECTING
    _wait(lambda: controller.state is ServerState.DISCONNECTED)

    assert worker.closed
    assert supervisor.stops == [True]
    # A disconnected service stays disconnected; the reconnect loop must not
    # rebuild the session it was just asked to drop.
    time.sleep(0.15)
    assert controller.state is ServerState.DISCONNECTED
    assert len(workers) == 1
    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "Scene stopped" in messages
    controller.close()


def test_stop_world_requires_an_idle_service():
    workers = []
    controller = _ready_controller(
        workers, simulator=FakeSupervisor(running=True)
    )
    task = controller.submit("fly")
    _wait(lambda: controller.state is ServerState.RUNNING)

    with pytest.raises(BackendUnavailable) as error:
        controller.stop_world()
    assert error.value.state is ServerState.RUNNING

    workers[0].emit(
        WorkerEventKind.FINISHED,
        task_id=task.task_id,
        success=True,
        message="done",
    )
    _wait(lambda: controller.state is ServerState.IDLE)
    controller.close()


def test_stop_world_leaves_a_stack_it_does_not_own():
    workers = []
    supervisor = FakeSupervisor(running=False)
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()
    controller.request_world("city")
    _wait(lambda: controller.state is ServerState.IDLE)
    supervisor.preflight_error = SimulatorNotOwned("no pidfiles")

    controller.stop_world()
    _wait(lambda: controller.state is ServerState.DISCONNECTED)

    assert supervisor.stops == []
    assert "keeps running" in (controller.status.sim_error or "")
    controller.close()


def test_closing_the_service_stops_the_stack_it_started():
    """Ctrl-C used to stop the service and leave UE rendering on the GPU.

    The next boot cleaned it up, which reads as a leak from the console. The flight
    console owns what it launched, and so does this service.
    """
    workers = []
    supervisor = FakeSupervisor(running=False)
    controller = _controller(workers, simulator=supervisor, autostart_session=False)
    controller.start()
    controller.request_world("city")
    _wait(lambda: controller.state is ServerState.IDLE)
    supervisor.stops.clear()          # only the shutdown stop is under test here

    controller.close()

    assert supervisor.stops == [True]
    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "Shutting down: stopping the simulator" in messages


def test_closing_without_a_stack_stops_nothing():
    workers = []
    supervisor = FakeSupervisor(running=False)
    controller = _controller(workers, simulator=supervisor, autostart_session=False)
    controller.start()
    supervisor.stops.clear()

    controller.close()

    assert supervisor.stops == []


def test_closing_does_not_kill_a_stack_it_cannot_prove_it_started():
    workers = []

    class Foreign(FakeSupervisor):
        def stop(self):
            raise SimulatorNotOwned("port 20010 belongs to unmanaged PID 42")

    supervisor = Foreign(running=True)
    controller = _controller(workers, simulator=supervisor, autostart_session=False)
    controller.start()
    supervisor.stops.clear()

    controller.close()                # must not raise out of the shutdown hook

    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "Leaving a simulator this service did not start" in messages


def test_scene_start_attaches_to_an_unowned_running_stack():
    workers = []
    supervisor = FakeSupervisor(
        running=True, preflight_error=SimulatorNotOwned("no pidfiles")
    )
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()

    assert controller.request_world("city") is ServerState.CONNECTING
    _wait(lambda: controller.state is ServerState.IDLE)

    assert supervisor.launches == []
    assert supervisor.restarts == []
    assert supervisor.stops == []
    assert supervisor.preflight_calls == 1
    controller.close()


def test_scene_start_launches_a_fresh_stack_after_startup_cleanup():
    workers = []
    supervisor = FakeSupervisor(running=True)
    controller = _controller(
        workers, simulator=supervisor, autostart_session=False
    )
    controller.start()

    assert controller.request_world("suburb") is ServerState.CONNECTING
    _wait(lambda: controller.state is ServerState.IDLE)

    assert supervisor.launches == ["suburb"]
    assert supervisor.restarts == []
    assert supervisor.stops == [True]
    assert supervisor.preflight_calls == 1
    assert controller.status.map == "suburb"
    controller.close()


def test_scene_start_failure_returns_to_disconnected_for_retry():
    workers = []
    supervisor = FakeSupervisor(running=False, launch_fail_times=2)
    controller = _controller(
        workers,
        simulator=supervisor,
        autostart_session=False,
        restart_attempts=2,
    )
    controller.start()
    controller.request_world("city")
    _wait(lambda: len(supervisor.launches) == 2)
    _wait(lambda: controller.state is ServerState.DISCONNECTED)

    assert supervisor.launches == ["city", "city"]
    assert controller.status.sim_error is not None
    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "Map start failed" in messages

    assert controller.request_world("city") is ServerState.CONNECTING
    _wait(lambda: controller.state is ServerState.IDLE)
    assert supervisor.launches == ["city", "city", "city"]
    controller.close()


def test_stop_requests_worker_stop_then_returns_idle():
    workers = []
    controller = _controller(workers)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    worker = workers[0]
    task = controller.submit("fly")
    _wait(lambda: worker.tasks)

    assert controller.stop() is ServerState.STOPPING
    _wait(lambda: worker.stops == 1)
    assert controller.status.state is ServerState.STOPPING

    worker.emit(
        WorkerEventKind.FINISHED,
        task_id=task.task_id,
        success=False,
        message="cancelled: task was cancelled",
    )
    _wait(lambda: controller.state is ServerState.IDLE)
    assert controller.status.last_result == "cancelled: task was cancelled"
    controller.close()


def test_home_is_forwarded_only_while_idle():
    workers = []
    controller = _ready_controller(workers)
    assert controller.home() == "rover"
    _wait(lambda: workers[0].homes == ["rover"])
    assert any(
        "Returning the rover to its start position" in item.message
        for item in controller.logs.read(0).items
    )

    controller.submit("go")
    _wait(lambda: workers[0].tasks)
    with pytest.raises(BackendUnavailable):
        controller.home()
    assert workers[0].homes == ["rover"]
    controller.close()


def test_home_is_unavailable_before_the_worker_is_ready():
    controller = _controller([])
    with pytest.raises(BackendUnavailable):
        controller.home()
    controller.close()


def test_home_routes_to_the_drone():
    workers = []
    controller = _ready_controller(workers)
    assert controller.home("drone") == "drone"
    _wait(lambda: workers[0].homes == ["drone"])
    controller.close()


def test_home_defaults_to_the_active_vehicle():
    workers = []
    controller = _ready_controller(workers)
    controller.submit("fly", "drone")
    _wait(lambda: workers[0].tasks)
    controller.stop()
    workers[0].emit(
        WorkerEventKind.FINISHED,
        task_id="task_1",
        success=False,
        message="cancelled",
    )
    _wait(lambda: controller.state is ServerState.IDLE)
    assert controller.home() == "drone"
    _wait(lambda: workers[0].homes == ["drone"])
    controller.close()


def test_home_rejects_an_unknown_vehicle():
    workers = []
    controller = _ready_controller(workers)
    with pytest.raises(ValueError, match="unknown vehicle"):
        controller.home("boat")
    assert workers[0].homes == []
    controller.close()


def test_activate_forwards_to_the_worker_and_updates_the_status():
    workers = []
    controller = _ready_controller(workers)
    assert controller.activate("drone") == "drone"
    assert controller.status.vehicle == "drone"
    assert controller.status.view_mode == "drone_chase"
    _wait(lambda: workers[0].activations == ["drone"])
    controller.close()


def test_activate_is_unavailable_during_a_task():
    workers = []
    controller = _ready_controller(workers)
    controller.submit("fly")
    _wait(lambda: workers[0].tasks)
    with pytest.raises(BackendUnavailable):
        controller.activate("drone")
    assert workers[0].activations == []
    controller.close()


def test_activate_rejects_an_unknown_vehicle():
    workers = []
    controller = _ready_controller(workers)
    with pytest.raises(ValueError, match="unknown vehicle"):
        controller.activate("boat")
    assert workers[0].activations == []
    controller.close()


def test_activate_can_repeat_the_current_vehicle():
    workers = []
    controller = _ready_controller(workers)
    assert controller.activate("rover") == "rover"
    _wait(lambda: workers[0].activations == ["rover"])
    controller.close()


def test_active_event_marks_the_chase_view_applied():
    workers = []
    controller = _ready_controller(workers)
    controller.activate("drone")
    _wait(lambda: workers[0].activations == ["drone"])
    workers[0].emit(WorkerEventKind.ACTIVE, vehicle="drone", success=True)
    _wait(lambda: controller._applied_view == "drone_chase")
    assert controller.status.view_mode == "drone_chase"
    controller.close()


def test_active_event_failure_restores_the_previous_selection():
    workers = []
    controller = _ready_controller(workers)
    controller.request_view("rover_front")
    _wait(lambda: workers[0].views == ["rover_front"])
    controller.activate("drone")
    assert controller.status.vehicle == "drone"
    assert controller.status.view_mode == "drone_chase"
    _wait(lambda: workers[0].activations == ["drone"])

    workers[0].emit(
        WorkerEventKind.ACTIVE,
        vehicle="drone",
        success=False,
        message="could not park the rover",
    )
    _wait(lambda: controller.status.vehicle == "rover")
    assert controller.status.view_mode == "rover_front"
    assert any(
        "could not park the rover" in item.message
        for item in controller.logs.read(0).items
    )
    controller.close()


def test_view_is_forwarded_while_idle():
    workers = []
    controller = _ready_controller(workers)
    assert controller.request_view("drone_chase") == ("drone_chase", False)
    _wait(lambda: workers[0].views == ["drone_chase"])
    assert controller.status.view_mode == "drone_chase"
    controller.close()


def test_view_is_deferred_during_a_task():
    workers = []
    controller = _ready_controller(workers)
    task = controller.submit("fly")
    _wait(lambda: workers[0].tasks)
    assert controller.request_view("drone_front") == ("drone_front", True)
    assert workers[0].views == []

    workers[0].emit(
        WorkerEventKind.FINISHED,
        task_id=task.task_id,
        success=True,
        message="reached",
    )
    _wait(lambda: controller.state is ServerState.IDLE)
    _wait(lambda: workers[0].views == ["drone_front"])
    controller.close()


def test_view_rejects_an_unknown_mode():
    workers = []
    controller = _ready_controller(workers)
    with pytest.raises(ValueError, match="unknown view mode"):
        controller.request_view("moon")
    assert workers[0].views == []
    controller.close()


def test_activate_sets_view_pending_until_active_confirms():
    workers = []
    controller = _ready_controller(workers)
    controller.activate("drone")
    assert controller.status.view_pending is True
    _wait(lambda: workers[0].activations == ["drone"])
    workers[0].emit(WorkerEventKind.ACTIVE, vehicle="drone", success=True)
    _wait(lambda: controller.status.view_pending is False)
    assert controller._applied_view == "drone_chase"
    controller.close()


def test_view_pending_clears_when_the_worker_confirms_the_view():
    workers = []
    controller = _ready_controller(workers)
    assert controller.request_view("drone_chase") == ("drone_chase", False)
    assert controller.status.view_pending is True
    _wait(lambda: workers[0].views == ["drone_chase"])
    workers[0].emit(WorkerEventKind.VIEW, vehicle="drone_chase", success=True)
    _wait(lambda: controller.status.view_pending is False)
    assert controller._applied_view == "drone_chase"
    controller.close()


def test_failed_view_restores_the_applied_view_while_idle():
    workers = []
    controller = _ready_controller(workers)
    controller.request_view("rover_front")
    _wait(lambda: workers[0].views == ["rover_front"])
    workers[0].emit(WorkerEventKind.VIEW, vehicle="rover_front", success=True)
    _wait(lambda: controller._applied_view == "rover_front")

    controller.request_view("drone_front")
    assert controller.status.view_mode == "drone_front"
    workers[0].emit(
        WorkerEventKind.VIEW,
        vehicle="drone_front",
        success=False,
        message="view change failed",
    )
    _wait(lambda: controller.status.view_mode == "rover_front")
    assert controller.status.view_pending is False
    controller.close()


def test_deferred_view_marks_pending_after_the_task_finishes():
    workers = []
    controller = _ready_controller(workers)
    task = controller.submit("fly")
    _wait(lambda: workers[0].tasks)
    assert controller.request_view("drone_front") == ("drone_front", True)
    assert controller.status.view_pending is False

    workers[0].emit(
        WorkerEventKind.FINISHED,
        task_id=task.task_id,
        success=True,
        message="reached",
    )
    _wait(lambda: workers[0].views == ["drone_front"])
    assert controller.status.view_pending is True
    workers[0].emit(WorkerEventKind.VIEW, vehicle="drone_front", success=True)
    _wait(lambda: controller.status.view_pending is False)
    controller.close()


def test_weather_merges_partial_updates():
    workers = []
    controller = _ready_controller(workers)
    merged = controller.request_weather({"rain": 0.5})
    assert set(merged) == set(WEATHER_FIELDS)
    assert merged["rain"] == 0.5
    assert merged["fog"] == 0.0
    _wait(lambda: len(workers[0].weathers) == 1)
    assert workers[0].weathers[0]["rain"] == 0.5

    second = controller.request_weather({"fog": 0.8})
    assert second["rain"] == 0.5
    assert second["fog"] == 0.8
    _wait(lambda: len(workers[0].weathers) == 2)
    assert workers[0].weathers[1]["fog"] == 0.8
    assert controller.status.weather["rain"] == 0.5
    controller.close()


def test_weather_rejects_invalid_values():
    workers = []
    controller = _ready_controller(workers)
    with pytest.raises(ValueError):
        controller.request_weather({"rain": 1.5})
    with pytest.raises(ValueError):
        controller.request_weather({"unknown": 0.5})
    assert workers[0].weathers == []
    controller.close()


def test_weather_is_unavailable_during_a_task():
    workers = []
    controller = _ready_controller(workers)
    controller.submit("fly")
    _wait(lambda: workers[0].tasks)
    with pytest.raises(BackendUnavailable):
        controller.request_weather({"rain": 0.5})
    assert workers[0].weathers == []
    controller.close()


def test_weather_is_replayed_but_the_view_resets_on_the_next_ready():
    workers = []
    controller = _ready_controller(workers)
    controller.request_view("drone_front")
    controller.request_weather({"rain": 0.4})
    _wait(lambda: workers[0].views == ["drone_front"])
    _wait(lambda: len(workers[0].weathers) == 1)

    workers[0].emit(
        WorkerEventKind.ERROR, message="ProjectAirSim session setup failed"
    )
    _wait(lambda: len(workers) == 2)
    _wait(lambda: controller.state is ServerState.IDLE)
    # A fresh session always starts on the default vehicle's chase camera, so
    # the stale selection must not be replayed onto it.
    assert workers[1].views == []
    assert controller.status.vehicle == "rover"
    assert controller.status.view_mode == "rover_chase"
    assert workers[1].weathers[-1]["rain"] == 0.4
    controller.close()


def test_world_switch_restarts_the_stack_between_workers():
    workers = []
    supervisor = FakeSupervisor()
    controller = _ready_controller(workers, simulator=supervisor)

    assert controller.request_world("suburb") is ServerState.CONNECTING
    assert controller.status.state is ServerState.CONNECTING
    assert controller.status.map_switching is True

    _wait(lambda: len(workers) == 2)
    assert workers[0].closed
    assert supervisor.restarts == ["suburb"]
    assert controller._test_selections[-1].map_id == "suburb"
    assert controller._test_selections[-1].scene_file == (
        "scene_neighborhood_dual.jsonc"
    )
    assert controller._test_selections[-1].sim_config_path == (
        "config/scenes/neighborhood"
    )

    _wait(lambda: controller.state is ServerState.IDLE)
    assert controller.status.map == "suburb"
    assert controller.status.map_switching is False
    controller.close()


def test_world_switch_to_the_current_map_is_a_noop():
    workers = []
    supervisor = FakeSupervisor()
    controller = _ready_controller(workers, simulator=supervisor)
    assert controller.request_world("city") is ServerState.IDLE
    assert supervisor.restarts == []
    assert len(workers) == 1
    controller.close()


def test_world_switch_rejects_an_unknown_map():
    workers = []
    supervisor = FakeSupervisor()
    controller = _ready_controller(workers, simulator=supervisor)
    with pytest.raises(ValueError, match="unknown map"):
        controller.request_world("moon")
    assert supervisor.restarts == []
    controller.close()


def test_simulator_owned_reflects_the_supervisor_preflight():
    controller = _controller([])
    assert controller.simulator_owned is False
    controller.close()

    supervisor = FakeSupervisor()
    controller = _controller([], simulator=supervisor)
    assert controller.simulator_owned is True
    assert supervisor.preflight_calls == 1
    controller.close()

    unowned = FakeSupervisor(preflight_error=SimulatorNotOwned("no pidfiles"))
    controller = _controller([], simulator=unowned)
    assert controller.simulator_owned is False
    controller.close()


def test_world_switch_requires_idle():
    workers = []
    supervisor = FakeSupervisor()
    controller = _ready_controller(workers, simulator=supervisor)
    controller.submit("fly")
    _wait(lambda: workers[0].tasks)
    with pytest.raises(BackendUnavailable):
        controller.request_world("suburb")
    assert supervisor.restarts == []
    controller.close()


def test_world_switch_rejects_an_unowned_simulator():
    workers = []
    supervisor = FakeSupervisor(preflight_error=RuntimeError("not ours"))
    controller = _ready_controller(workers, simulator=supervisor)
    with pytest.raises(ValueError, match="not ours"):
        controller.request_world("suburb")
    assert controller.state is ServerState.IDLE
    assert supervisor.restarts == []
    controller.close()


def test_world_switch_waits_for_worker_closed_before_restart():
    workers = []
    supervisor = FakeSupervisor()
    controller = _ready_controller(
        workers,
        simulator=supervisor,
        worker_class=SilentWorker,
        worker_shutdown_timeout_s=0.05,
    )
    assert controller.request_world("suburb") is ServerState.CONNECTING
    time.sleep(0.2)
    assert supervisor.restarts == []
    assert controller.state is ServerState.CONNECTING
    assert controller.status.map_switching is True
    controller.close()


def test_world_retry_succeeds_after_one_failure():
    workers = []
    supervisor = FakeSupervisor(fail_times=1)
    controller = _ready_controller(workers, simulator=supervisor)
    controller.request_world("suburb")
    _wait(lambda: len(workers) == 2)
    assert supervisor.restarts == ["suburb", "suburb"]
    _wait(lambda: controller.state is ServerState.IDLE)
    assert controller.status.map == "suburb"
    controller.close()


def test_world_failure_returns_to_disconnected_with_sim_error():
    workers = []
    supervisor = FakeSupervisor(fail_times=5)
    controller = _ready_controller(workers, simulator=supervisor)
    controller.request_world("suburb")
    _wait(lambda: controller.status.sim_error is not None)
    assert controller.state is ServerState.DISCONNECTED
    assert len(workers) == 1
    assert controller.status.map_switching is False
    assert "restart" in controller.status.sim_error
    controller.close()


def _stale_hub():
    """A console still holding the path the previous run was given."""
    hub = TrajectoryHub()
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    hub.publish_queue(
        instruction="drive to the red car",
        stage_id="stage_1",
        mode=MODE_CONTINUOUS,
        capacity=10,
        pose=start,
        items=(
            QueuedWaypoint(4, start, WorldPose(3.0, 4.0, 0.0, 90.0), 90.0),
            QueuedWaypoint(
                5,
                WorldPose(3.0, 4.0, 0.0, 90.0),
                WorldPose(6.0, 4.0, 0.0, 90.0),
                0.0,
            ),
        ),
        planning_calls=1,
        path_length_m=7.0,
    )
    return hub


def test_a_session_switch_clears_the_path_the_previous_run_left_on_the_console():
    """The vehicles go back to spawn; the drawn path has to go with them.

    A map, appearance-model or configuration switch rebuilds the session, and the
    simulator forgets the plan it was executing. The console does not: it kept
    publishing the old queue until the next task started, which is what made a
    switched session look like it was still on its way to the previous goal.
    """
    hub = _stale_hub()
    workers = []
    controller = _ready_controller(
        workers, vehicle_models=_models(), session_reset=hub.clear
    )
    assert len(hub.read().points) == 2

    controller.request_models({"drone": "hexarotor"})
    _wait(lambda: controller.state is ServerState.IDLE and len(workers) == 2)

    snapshot = hub.read()
    assert snapshot.active is False
    assert snapshot.points == ()
    controller.close()


def test_a_session_reset_that_raises_does_not_stall_the_restart():
    def explode():
        raise RuntimeError("console unreachable")

    workers = []
    controller = _ready_controller(
        workers, vehicle_models=_models(), session_reset=explode
    )
    controller.request_models({"drone": "hexarotor"})
    _wait(lambda: controller.state is ServerState.IDLE and len(workers) == 2)

    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "Session reset failed" in messages
    controller.close()


def _profiles():
    return (
        ProfileInfo("default", "世界模型规划器", "实机调校参数", "deepseek-flash"),
        ProfileInfo("vlm", "普通 VLM 基线", "通用导航参数", "deepseek-flash"),
    )


def test_profile_switch_rebuilds_the_session_without_touching_the_map():
    """Selecting a profile is a session rebuild, not a simulator restart."""
    workers = []
    controller = _ready_controller(
        workers, profiles=_profiles(), initial_profile="default"
    )
    assert controller.profile == "default"

    state = controller.request_profile("vlm")
    assert state is ServerState.CONNECTING
    _wait(lambda: controller.state is ServerState.IDLE and len(workers) == 2)

    assert controller.profile == "vlm"
    assert controller.status.profile == "vlm"
    assert controller.status.profiles == ("default", "vlm")
    assert controller.status.profile_switching is False
    # Same map and same scene file: only the session was assembled again.
    assert controller._test_selections[0] == controller._test_selections[1]
    controller.close()


def test_selecting_a_profile_twice_does_not_restart_anything():
    workers = []
    controller = _ready_controller(workers, profiles=_profiles())
    assert controller.request_profile("default") is ServerState.IDLE
    assert len(workers) == 1
    controller.close()


def test_profile_switch_requires_idle():
    workers = []
    controller = _ready_controller(workers, profiles=_profiles())
    controller.submit("fly")
    _wait(lambda: workers[0].tasks)
    with pytest.raises(BackendUnavailable):
        controller.request_profile("vlm")
    controller.close()


def test_a_profile_can_be_selected_before_a_scene_is_loaded():
    """With no session there is nothing to rebuild, so the choice waits."""
    workers = []
    controller = _controller(
        workers,
        simulator=FakeSupervisor(),
        autostart_session=False,
        profiles=_profiles(),
        initial_profile="default",
    )
    assert controller.state is ServerState.DISCONNECTED

    assert controller.request_profile("vlm") is ServerState.DISCONNECTED
    assert controller.profile == "vlm"
    assert controller.status.profile_switching is False
    assert workers == [], "selecting a profile must not build a session"
    assert controller._test_selections == []

    messages = "\n".join(item.message for item in controller.logs.read(0).items)
    assert "scene starts" in messages

    controller.start()
    assert controller.request_world("city") is ServerState.CONNECTING
    _wait(lambda: controller.state is ServerState.IDLE)
    assert controller.profile == "vlm", "the scene starts from the chosen profile"
    controller.close()


def test_profile_switch_rejects_a_name_that_is_not_installed():
    workers = []
    controller = _ready_controller(workers, profiles=_profiles())
    with pytest.raises(ValueError, match="expected one of"):
        controller.request_profile("nope")
    assert controller.state is ServerState.IDLE
    assert len(workers) == 1
    controller.close()


def test_model_switch_reloads_the_worker_with_the_variant_scene():
    workers = []
    controller = _ready_controller(workers, vehicle_models=_models())
    assert controller.status.models == {"rover": "offroad", "drone": "quadrotor"}
    assert controller._test_selections[0] == MapSelection(
        "city", "scene_city_dual.jsonc", "config/scenes/city"
    )

    state = controller.request_models({"drone": "hexarotor"})
    assert state is ServerState.CONNECTING
    assert controller.status.models["drone"] == "hexarotor"
    _wait(lambda: controller.state is ServerState.IDLE and len(workers) == 2)

    assert controller._test_selections[1] == MapSelection(
        "city", "scene_city_dual_hexarotor.jsonc", "config/scenes/city"
    )
    assert controller.status.models == {"rover": "offroad", "drone": "hexarotor"}
    assert controller.status.models_switching is False
    controller.close()


def test_model_switch_keeps_the_selection_across_a_map_switch():
    workers = []
    controller = _ready_controller(
        workers, vehicle_models=_models(), simulator=FakeSupervisor()
    )
    controller.request_models({"drone": "hexarotor"})
    _wait(lambda: controller.state is ServerState.IDLE and len(workers) == 2)

    controller.request_world("suburb")
    _wait(lambda: controller.state is ServerState.IDLE and len(workers) == 3)
    assert controller._test_selections[2] == MapSelection(
        "suburb",
        "scene_neighborhood_dual_hexarotor.jsonc",
        "config/scenes/neighborhood",
    )
    assert controller.status.models == {"rover": "offroad", "drone": "hexarotor"}
    controller.close()


def test_model_switch_to_the_current_selection_is_a_noop():
    workers = []
    controller = _ready_controller(workers, vehicle_models=_models())
    state = controller.request_models({"drone": "quadrotor"})
    assert state is ServerState.IDLE
    time.sleep(0.05)
    assert len(workers) == 1
    controller.close()


def test_model_switch_rejects_an_unknown_model():
    workers = []
    controller = _ready_controller(workers, vehicle_models=_models())
    with pytest.raises(ValueError, match="unknown drone model"):
        controller.request_models({"drone": "trirotor"})
    with pytest.raises(ValueError, match="unknown vehicle"):
        controller.request_models({"boat": "offroad"})
    assert controller.state is ServerState.IDLE
    assert len(workers) == 1
    controller.close()


def test_model_switch_is_unavailable_during_a_task():
    workers = []
    controller = _ready_controller(workers, vehicle_models=_models())
    controller.submit("fly")
    _wait(lambda: workers[0].tasks)
    with pytest.raises(BackendUnavailable) as error:
        controller.request_models({"drone": "hexarotor"})
    assert error.value.state is ServerState.RUNNING
    controller.close()


def test_model_switch_is_unavailable_without_a_catalog():
    workers = []
    controller = _ready_controller(workers)
    with pytest.raises(ValueError, match="does not configure"):
        controller.request_models({"drone": "hexarotor"})
    assert controller.status.models is None
    controller.close()


def test_worker_error_reconnects_with_a_new_worker():
    workers = []
    controller = _controller(workers)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    workers[0].emit(
        WorkerEventKind.ERROR, message="ProjectAirSim session setup failed"
    )
    _wait(lambda: len(workers) == 2)
    _wait(lambda: controller.state is ServerState.IDLE)
    assert workers[1].started
    assert any(
        "ProjectAirSim session setup failed" in item.message
        for item in controller.logs.read(0).items
    )
    controller.close()


def test_close_shuts_down_the_worker():
    workers = []
    controller = _controller(workers)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    controller.close()
    assert controller.state is ServerState.STOPPING
    assert workers[0].closed


def test_submit_returns_task_ids_in_order():
    workers = []
    controller = _controller(workers)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    first = controller.submit("one")
    _wait(lambda: workers[0].tasks)
    workers[0].emit(
        WorkerEventKind.FINISHED, task_id=first.task_id, success=True
    )
    _wait(lambda: controller.state is ServerState.IDLE)
    second = controller.submit("two")
    assert (first.task_id, second.task_id) == ("task_1", "task_2")
    controller.close()


def test_duplicate_finished_event_is_ignored():
    workers = []
    controller = _controller(workers)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    task = controller.submit("one")
    _wait(lambda: workers[0].tasks)
    workers[0].emit(
        WorkerEventKind.FINISHED, task_id=task.task_id, success=True
    )
    _wait(lambda: controller.state is ServerState.IDLE)
    workers[0].emit(
        WorkerEventKind.FINISHED,
        task_id=task.task_id,
        success=False,
        message="late duplicate",
    )
    time.sleep(0.1)
    assert controller.state is ServerState.IDLE
    assert controller.status.last_result == "reached"
    controller.close()


def test_task_submission_requires_non_empty_instruction():
    controller = _controller([])
    with pytest.raises(ValueError):
        controller.submit("   ")


def test_submit_routes_to_the_requested_vehicle():
    workers = []
    controller = _controller(workers)
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    task = controller.submit("fly to the target", "drone")
    _wait(lambda: workers[0].tasks)
    assert task.vehicle == "drone"
    assert workers[0].tasks[0].vehicle == "drone"
    assert controller.status.vehicle == "drone"
    assert controller.status.vehicles == ("rover", "drone")
    controller.close()


def test_submit_defaults_to_the_configured_vehicle():
    workers = []
    controller = _controller(workers, default_vehicle="drone")
    controller.start()
    _wait(lambda: controller.state is ServerState.IDLE)
    task = controller.submit("fly")
    assert task.vehicle == "drone"
    assert controller.status.vehicle == "drone"
    controller.close()


def test_submit_rejects_an_unknown_vehicle():
    controller = _controller([])
    with pytest.raises(ValueError, match="unknown vehicle"):
        controller.submit("fly", "boat")
    controller.close()
