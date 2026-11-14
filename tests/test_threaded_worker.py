import time

import pytest

from planner.adapters.runtime.worker import ThreadedMissionWorker
from planner.domain.server import TaskSubmission, WorkerEventKind
from planner.domain.weather import WEATHER_CATALOG
from planner.domain.result import (
    CompletionBasis,
    MissionResult,
    StageResult,
    StageSummary,
)


def _success() -> MissionResult:
    return MissionResult(
        True,
        (
            StageResult(
                "stage_1",
                True,
                summary=StageSummary(
                    "completed",
                    "test",
                    CompletionBasis.LOCAL_MEASUREMENT,
                    True,
                ),
            ),
        ),
    )


def _failure(error: str) -> MissionResult:
    return MissionResult(False, (), error=error)


class FakeSession:
    def __init__(
        self,
        result: MissionResult | None = None,
        *,
        block_until_stop: bool = False,
    ):
        self.result = result or _success()
        self.block_until_stop = block_until_stop
        self.tasks = []
        self.closed = False
        self.skip_landing = None
        self.saw_stop = False
        self.idle_ticks = 0
        self.homes = []
        self.activations = []
        self.views = []
        self.weathers = []
        self.fail_view = False
        self.fail_activate = False

    def idle_tick(self):
        self.idle_ticks += 1

    def return_home(self, vehicle=None):
        self.homes.append(vehicle)

    def activate_vehicle(self, vehicle=None):
        if self.fail_activate:
            raise RuntimeError("activation failed")
        self.activations.append(vehicle)
        return vehicle

    def set_view_mode(self, mode):
        if self.fail_view:
            raise RuntimeError("view switch failed")
        self.views.append(mode)

    def set_weather(self, fields):
        self.weathers.append(dict(fields))

    def weather_catalog(self):
        return WEATHER_CATALOG

    def run(self, task, *, stop_event, log):
        self.tasks.append(task)
        log(f"running {task.task_id}")
        if self.block_until_stop:
            while not stop_event.wait(0.01):
                pass
            self.saw_stop = True
            return _failure("cancelled: task was cancelled")
        return self.result

    def close(self, *, skip_landing=False):
        self.closed = True
        self.skip_landing = skip_landing


def _collect(worker, predicate, timeout=2.0):
    items = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        event = worker.poll()
        if event is None:
            time.sleep(0.01)
            continue
        items.append(event)
        if predicate(event):
            return items
    raise AssertionError(f"event not seen; got {items}")


def _value(kind):
    return lambda event: event.kind is kind


def test_worker_emits_ready_and_runs_a_task():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))

    task = TaskSubmission("task_1", "fly to the red car")
    worker.submit(task)
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    finished = events[-1]
    assert finished.task_id == "task_1"
    assert finished.success is True
    assert finished.message.startswith(
        "Mission completion confirmed by task-specific evidence:"
    )
    assert session.tasks == [task]
    worker.shutdown()
    assert session.closed
    _collect(worker, _value(WorkerEventKind.CLOSED))


def test_stop_cancels_the_running_task():
    session = FakeSession(block_until_stop=True)
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.submit(TaskSubmission("task_1", "fly"))

    deadline = time.monotonic() + 2.0
    while not session.tasks and time.monotonic() < deadline:
        time.sleep(0.01)
    assert session.tasks

    worker.stop()
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    assert events[-1].success is False
    assert session.saw_stop
    logs = [
        event.message
        for event in events
        if event.kind is WorkerEventKind.LOG
    ]
    assert any("Mission failed: cancelled:" in message for message in logs)
    assert any(
        "Stop confirmed for task_1" in message for message in logs
    )
    worker.shutdown()


def test_stop_before_first_task_is_not_lost():
    session = FakeSession(block_until_stop=True)
    worker = ThreadedMissionWorker(lambda log: session)
    worker.stop()
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.submit(TaskSubmission("task_1", "fly"))
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    assert events[-1].success is False
    assert session.saw_stop
    worker.shutdown()


def test_failed_status_is_reported_as_unsuccessful():
    session = FakeSession(_failure("timeout: no target"))
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.submit(TaskSubmission("task_1", "fly"))
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    assert events[-1].success is False
    assert events[-1].message.startswith("Mission failed: timeout:")
    worker.shutdown()


def test_session_factory_failure_emits_error():
    def factory(log):
        raise RuntimeError("ProjectAirSim server did not become ready")

    worker = ThreadedMissionWorker(factory)
    worker.start()
    events = _collect(worker, _value(WorkerEventKind.ERROR))
    assert "ProjectAirSim server did not become ready" in events[-1].message
    worker.shutdown()
    _collect(worker, _value(WorkerEventKind.CLOSED))


def test_session_exception_is_reported_and_worker_survives():
    class ExplodingSession(FakeSession):
        def run(self, task, *, stop_event, log):
            if not self.tasks:
                self.tasks.append(task)
                raise RuntimeError("depth sampling failed")
            return super().run(task, stop_event=stop_event, log=log)

    session = ExplodingSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.submit(TaskSubmission("task_1", "fly"))
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    assert events[-1].success is False
    assert "Task rejected:" in events[-1].message
    assert "depth sampling failed" in events[-1].message

    worker.submit(TaskSubmission("task_2", "fly again"))
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    assert events[-1].task_id == "task_2"
    assert events[-1].success is True
    worker.shutdown()


def test_worker_ticks_the_world_while_idle():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))

    deadline = time.monotonic() + 1.0
    while session.idle_ticks < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert session.idle_ticks >= 3
    worker.shutdown()


def test_worker_shutdown_closes_the_session():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.shutdown()
    assert session.closed
    assert session.skip_landing is False
    _collect(worker, _value(WorkerEventKind.CLOSED))
    with pytest.raises(AssertionError):
        _collect(worker, _value(WorkerEventKind.READY), timeout=0.1)


def test_worker_shutdown_skips_landing_when_the_scene_is_replaced():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.shutdown(skip_landing=True)
    assert session.closed
    assert session.skip_landing is True


def test_home_command_reaches_session():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.home("drone")

    deadline = time.monotonic() + 1.0
    while not session.homes and time.monotonic() < deadline:
        time.sleep(0.01)
    assert session.homes == ["drone"]
    worker.shutdown()


def test_view_and_weather_commands_reach_session():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.set_view_mode("drone_front")
    worker.set_weather({"rain": 0.5})

    deadline = time.monotonic() + 1.0
    while (
        not session.weathers or not session.views
    ) and time.monotonic() < deadline:
        time.sleep(0.01)
    assert session.views == ["drone_front"]
    assert session.weathers == [{"rain": 0.5}]
    worker.shutdown()


def test_view_command_emits_view_events():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.set_view_mode("drone_front")
    events = _collect(worker, _value(WorkerEventKind.VIEW))
    assert events[-1].vehicle == "drone_front"
    assert events[-1].success is True
    assert session.views == ["drone_front"]
    worker.shutdown()


def test_failed_view_change_emits_view_failure():
    session = FakeSession()
    session.fail_view = True
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.set_view_mode("drone_front")
    events = _collect(worker, _value(WorkerEventKind.VIEW))
    assert events[-1].vehicle == "drone_front"
    assert events[-1].success is False
    assert "view switch failed" in events[-1].message
    worker.shutdown()


def test_ready_emits_the_weather_catalog():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    events = _collect(worker, _value(WorkerEventKind.WEATHER))
    assert events[-1].payload is WEATHER_CATALOG
    worker.shutdown()


def test_failed_view_change_keeps_the_worker_alive():
    session = FakeSession()
    session.fail_view = True
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))
    worker.set_view_mode("drone_front")
    _collect(
        worker,
        lambda event: (
            event.kind is WorkerEventKind.LOG
            and "view change failed" in event.message
        ),
    )

    worker.submit(TaskSubmission("task_1", "fly"))
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    assert events[-1].success is True
    worker.shutdown()


def test_activate_command_reaches_session_and_emits_active():
    session = FakeSession()
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))

    worker.activate("drone")
    events = _collect(worker, _value(WorkerEventKind.ACTIVE))
    assert events[-1].vehicle == "drone"
    assert events[-1].success is True
    assert session.activations == ["drone"]
    worker.shutdown()


def test_failed_activation_keeps_the_worker_alive():
    session = FakeSession()
    session.fail_activate = True
    worker = ThreadedMissionWorker(lambda log: session)
    worker.start()
    _collect(worker, _value(WorkerEventKind.READY))

    worker.activate("drone")
    events = _collect(worker, _value(WorkerEventKind.ACTIVE))
    assert events[-1].vehicle == "drone"
    assert events[-1].success is False
    logs = [
        event.message
        for event in events
        if event.kind is WorkerEventKind.LOG
    ]
    assert any("vehicle activation failed" in message for message in logs)

    worker.submit(TaskSubmission("task_1", "fly"))
    events = _collect(worker, _value(WorkerEventKind.FINISHED))
    assert events[-1].success is True
    worker.shutdown()
