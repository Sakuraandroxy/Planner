"""The parent/child protocol: encoding, decoding, and one real round trip.

The interesting failure these guard against is not a crash; it is a silent stall.
``ServerController`` waits for a ``READY`` event, and if an event is lost -- dropped
because a library printed to the protocol channel, or never flushed because the child
buffered it -- the console sits in ``connecting`` while a healthy simulator flies
nothing. So the events travel on their own file descriptor, and every parse problem
becomes an explicit error instead of a missing event.
"""

import json
import sys
import threading
import time

import pytest

from planner.domain.server import TaskSubmission, WorkerEvent, WorkerEventKind
from planner.simulator.subprocess_worker import (
    ChildProcessError,
    SubprocessMissionWorker,
    decode_event,
    encode_command,
)


def test_command_round_trip():
    line = encode_command("task", task_id="t1", instruction="向前飞三米", vehicle="drone")
    assert line.endswith("\n")
    payload = json.loads(line)
    assert payload == {"cmd": "task", "task_id": "t1",
                       "instruction": "向前飞三米", "vehicle": "drone"}


def test_event_round_trip_keeps_the_optional_fields():
    event = WorkerEvent(WorkerEventKind.FINISHED, task_id="t1", message="done",
                        success=True, payload={"a": 1}, vehicle="drone", level="WARN")
    decoded = decode_event(json.dumps({"kind": event.kind.value, "task_id": "t1",
                                       "message": "done", "success": True,
                                       "payload": {"a": 1}, "vehicle": "drone",
                                       "level": "WARN"}))
    assert decoded.kind is WorkerEventKind.FINISHED
    assert (decoded.task_id, decoded.message, decoded.success) == ("t1", "done", True)
    assert (decoded.payload, decoded.vehicle, decoded.level) == ({"a": 1}, "drone", "WARN")


def test_a_non_protocol_line_is_an_error_not_a_skip():
    """A banner from some library must surface as an error, not vanish."""
    with pytest.raises(ChildProcessError, match="non-JSON line"):
        decode_event("libEGL warning: failed to open /dev/dri/card0")
    with pytest.raises(ChildProcessError, match="unknown event kind"):
        decode_event('{"kind": "telepathy"}')
    with pytest.raises(ChildProcessError, match="malformed event"):
        decode_event("[1, 2, 3]")


def test_submit_encodes_the_submission_fields_the_child_needs():
    from planner.config import default_config_path

    worker = SubprocessMissionWorker(
        python=sys.executable, module="planner.simulator.mission_child",
        config_path=str(default_config_path()), map_id="blocks", project_dir=".",
    )
    sent = []
    worker._stdin = type("W", (), {"write": staticmethod(lambda text: sent.append(text)),
                                   "flush": staticmethod(lambda: None)})()
    worker.submit(TaskSubmission(task_id="t9", instruction="左转", vehicle="drone"))
    assert json.loads(sent[0])["task_id"] == "t9"
    assert json.loads(sent[0])["cmd"] == "task"


def test_shutdown_waits_for_the_child_to_confirm(tmp_path):
    """The ordering bug that made every UE4 shutdown log an error.

    ``ServerController`` calls ``shutdown()`` and then only looks at what is already in
    the queue. Over a pipe the confirmation arrived after the call returned, so the
    console reported "Worker did not confirm shutdown within the timeout" against a
    child that had closed cleanly.
    """
    worker = SubprocessMissionWorker(
        python=sys.executable, module="planner.simulator.mission_child",
        config_path=str(tmp_path / "base.yaml"), map_id="blocks", project_dir=".",
        shutdown_timeout_s=5.0,
    )
    worker._process = type("P", (), {"poll": lambda self: None})()
    sent = []
    worker._stdin = type("W", (), {"write": staticmethod(lambda text: sent.append(text)),
                                   "flush": staticmethod(lambda: None)})()
    threading.Thread(
        target=lambda: (time.sleep(0.3),
                        worker._events.put(WorkerEvent(WorkerEventKind.CLOSED))),
        daemon=True,
    ).start()

    t0 = time.monotonic()
    worker.shutdown()
    waited = time.monotonic() - t0

    assert waited >= 0.25, waited                     # it really waited
    assert json.loads(sent[0])["cmd"] == "shutdown"
    event = worker.poll()                             # still deliverable to _drain
    assert event is not None and event.kind is WorkerEventKind.CLOSED


def test_shutdown_gives_up_on_the_bound_not_forever(tmp_path):
    """A wedged child must not wedge the service: the wait is bounded."""
    worker = SubprocessMissionWorker(
        python=sys.executable, module="planner.simulator.mission_child",
        config_path=str(tmp_path / "base.yaml"), map_id="blocks", project_dir=".",
        shutdown_timeout_s=0.4,
    )
    worker._process = type("P", (), {"poll": lambda self: None})()
    worker._stdin = type("W", (), {"write": staticmethod(lambda text: None),
                                   "flush": staticmethod(lambda: None)})()
    t0 = time.monotonic()
    worker.shutdown()
    assert 0.3 < time.monotonic() - t0 < 2.0
    assert worker.poll() is None


def test_the_weather_catalog_survives_the_pipe():
    """``GET /api/weather`` used to answer 500 on the UE4 path only.

    The catalog is a domain dataclass. The generic encoder turned it into nested dicts,
    the threaded worker kept passing the object, and the endpoint read
    ``catalog.fields`` off whichever arrived -- so the same request worked for UE5 and
    raised "'dict' object has no attribute 'fields'" for UE4.
    """
    from planner.domain.weather import WEATHER_CATALOG

    line = json.dumps({
        "kind": "weather",
        "payload": WEATHER_CATALOG.to_payload(),
    }, ensure_ascii=False)
    event = decode_event(line)
    assert isinstance(event.payload, type(WEATHER_CATALOG))
    assert [field.id for field in event.payload.fields] == [
        f.id for f in WEATHER_CATALOG.fields
    ]
    assert event.payload.preset("heavy_rain") is not None


def test_a_mangled_weather_payload_is_a_protocol_error():
    from planner.simulator.subprocess_worker import ChildProcessError

    with pytest.raises(ChildProcessError, match="weather payload"):
        decode_event('{"kind": "weather", "payload": {"fields": [], "presets": []}}')


def test_a_trajectory_snapshot_reaches_the_hub_and_not_the_event_queue():
    """The child's task loop cannot hold the parent's hub, so it relays snapshots.

    They must land in the hub the HTTP thread reads, and must not be queued as
    events: nothing upstream polls for them, and a sliding-window planner can
    publish several times per control period -- that would starve READY and
    FINISHED out of the queue that carries them.
    """
    from planner.domain.trajectory_telemetry import (
        QueueSnapshot,
        TrajectoryPointSnapshot,
        TrajectorySnapshot,
        VehicleSnapshot,
        snapshot_to_payload,
    )

    class Sink:
        def __init__(self):
            self.adopted = []

        def adopt(self, snapshot):
            self.adopted.append(snapshot)

    sink = Sink()
    worker = SubprocessMissionWorker(
        python=sys.executable,
        module="unused",
        config_path="unused",
        map_id="blocks",
        project_dir=".",
        trajectory_sink=sink,
    )
    snapshot = TrajectorySnapshot(
        active=True,
        instruction="Fly to the red car. 飞到红色汽车那里。",
        stage_id="stage_1",
        mode="stopped_batch",
        queue=QueueSnapshot(1, 4),
        vehicle=VehicleSnapshot(0.0, 0.0, -1.5, 0.0),
        planning_calls=2,
        path_length_m=4.0,
        points=(
            TrajectoryPointSnapshot(
                0, 3, 1.0, 2.0, -3.0, 10.0, 5.0, 1.2, 0.9
            ),
        ),
        revision=7,
    )
    line = json.dumps(
        {"kind": "trajectory", "payload": snapshot_to_payload(snapshot)},
        ensure_ascii=False,
    )

    class _Stream:
        def __init__(self, lines):
            self._items = iter(lines)

        def readline(self):
            return next(self._items, "")

    class _Process:
        stdout = _Stream([])
        stderr = _Stream([])

        @staticmethod
        def wait():
            return 0

    worker._event_stream = _Stream([line + "\n"])
    worker._process = _Process()
    worker._pump()

    assert len(sink.adopted) == 1
    adopted = sink.adopted[0]
    assert isinstance(adopted, TrajectorySnapshot)
    assert adopted.points[0].waypoint_id == 3
    assert adopted.instruction.startswith("Fly to the red car")
    queued = []
    while (event := worker.poll()) is not None:
        queued.append(event.kind)
    assert WorkerEventKind.TRAJECTORY not in queued, (
        "a snapshot must not occupy the event queue the console waits on"
    )


def test_a_mangled_trajectory_payload_is_a_protocol_error():
    from planner.simulator.subprocess_worker import ChildProcessError

    with pytest.raises(ChildProcessError, match="trajectory snapshot"):
        decode_event('{"kind": "trajectory", "payload": {"active": true}}')


def test_the_child_reports_its_own_setup_failure_over_the_pipe():
    """A real process, a real pipe, and the failure this machine actually produces.

    The AirSim client is deliberately absent from the environment running the tests
    (it lives in ``vln-ue4``; see docs/DEPLOYMENT.md 2), so the child cannot build a
    UE4 session. What matters is that the parent learns that through the protocol and
    then sees the child exit -- not that it waits for a READY nobody can send.
    """
    from pathlib import Path

    from planner.config import default_config_path, load_config

    config_path = default_config_path().resolve()
    config = load_config(config_path)
    family = config.families.require("airsim")
    assert family.runtime_python, (
        "the airsim family must name an interpreter, or UE4 sessions would be built "
        "in the HTTP process, which cannot import the AirSim client"
    )

    worker = SubprocessMissionWorker(
        python=sys.executable,
        module="planner.simulator.mission_child",
        config_path=str(config_path),
        map_id="blocks",
        # The repository root is what holds ``planner/``; the child imports from there.
        # It comes from this file's own location, not from the config path: a profile
        # directory sits an arbitrary depth below the root, so counting parents would
        # break the moment a profile is added deeper.
        project_dir=str(Path(__file__).resolve().parents[1]),
    )
    worker.start()
    deadline = time.time() + 90
    kinds, messages = [], []
    while time.time() < deadline and WorkerEventKind.CLOSED not in kinds:
        event = worker.poll()
        if event is None:
            time.sleep(0.1)
            continue
        kinds.append(event.kind)
        messages.append(f"{event.kind.value}: {event.message[:120]}")
    worker.terminate()

    assert WorkerEventKind.ERROR in kinds, messages
    assert WorkerEventKind.CLOSED in kinds, messages
    failure = " ".join(m for k, m in zip(kinds, messages) if k is WorkerEventKind.ERROR)
    assert "AirSim" in failure, failure
