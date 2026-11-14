"""The child must answer a stop while a task is still flying.

Commands and tasks share one loop in ``mission_child``, so a ``stop`` that goes through
the queue is only reached after the flight it was meant to cancel. Measured on Blocks:
``/api/stop`` six seconds into an eight-metre leg, and the stage still reported
``completed`` after 24.4s. ``ThreadedMissionWorker`` sets its event directly, which is
the behaviour the console is written against; the child now does the same on its reader
thread.
"""

import io
import json
import queue
import sys

from planner.simulator.mission_child import MissionChild


class _SilentEmitter:
    def __init__(self):
        self.lines = []
        self.events = []

    def log(self, message, level="INFO"):
        self.lines.append((level, message))

    def event(self, kind, **fields):
        self.events.append((kind, fields))


def _child(monkeypatch):
    monkeypatch.setattr("planner.simulator.mission_child.Emitter", _SilentEmitter)
    child = MissionChild("config/base.yaml", "blocks")
    return child


def test_a_stop_is_never_queued_behind_the_command_loop(monkeypatch):
    """Two rules in one: the stop is handled out of band, and it names real work.

    ``stop`` must not sit in the queue behind the flight it cancels (the original bug),
    and with nothing flying it must not be remembered for later either. Both are read
    here as: nothing queued, nothing set, one line in the log saying it was ignored.
    """
    child = _child(monkeypatch)
    lines = [json.dumps({"cmd": "stop"}), ""]
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n".join(lines)))
    child._reader()
    assert not child._stop_event.is_set()
    assert child._stop_requested is False
    # Only the EOF marker belongs in the queue; the stop must not have queued.
    rest = []
    while not child._commands.empty():
        rest.append(child._commands.get_nowait()["cmd"])
    assert rest == ["terminate"], rest
    assert any("ignored" in m for _, m in child.emitter.lines), child.emitter.lines


def test_a_parent_closing_the_pipe_still_asks_for_termination(monkeypatch):
    child = _child(monkeypatch)
    monkeypatch.setattr(sys, "stdin", io.StringIO(""))
    child._reader()
    command = child._commands.get_nowait()
    assert command["cmd"] == "terminate"


def test_an_unparseable_command_is_reported_not_swallowed(monkeypatch):
    child = _child(monkeypatch)
    monkeypatch.setattr(sys, "stdin", io.StringIO("{not json\n\n"))
    child._reader()
    queued = []
    while not child._commands.empty():
        queued.append(child._commands.get_nowait()["cmd"])
    assert queued == ["terminate"], queued
    assert any("unparseable" in m for _, m in child.emitter.lines)


class _StubSession:
    """Stands in for a mission session: records what it was asked to run."""

    def __init__(self):
        self.seen = []

    def run(self, task, stop_event=None, log=None):
        from planner.domain.result import MissionResult
        self.seen.append((task.task_id, stop_event.is_set() if stop_event else None))
        return MissionResult(success=True, stages=(), error=None)

    def close(self, *, skip_landing=False):
        pass


def test_a_finished_task_leaves_the_next_one_flyable(monkeypatch):
    """Two things the child broke at once on real hardware.

    An edit put ``self._session = None`` at the end of ``_clear_running``, so every
    completed task wiped the session and the next instruction died with
    "'NoneType' object has no attribute 'run'" (at mission_child.py:254). Separately,
    a stop flag left over from an earlier cancellation made the next task start already
    cancelled. Neither shows up unless a second task runs in the same child, which is
    exactly what the console does.
    """
    child = _child(monkeypatch)
    session = _StubSession()
    child._session = session

    child._run_task({"cmd": "task", "task_id": "t1", "instruction": "向前飞两米"})
    # A stop with nothing flying must not be deferred to the next instruction.
    monkeypatch.setattr(sys, "stdin", io.StringIO(json.dumps({"cmd": "stop"}) + "\n"))
    child._reader()
    child._run_task({"cmd": "task", "task_id": "t2", "instruction": "向右转90度"})

    assert child._session is session, "a finished task must not drop the session"
    assert session.seen == [("t1", False), ("t2", False)], session.seen
    assert child._task_running is None
    assert any("ignored" in m for _, m in child.emitter.lines), child.emitter.lines


def test_a_stop_that_arrives_while_a_task_is_queued_cancels_that_task(monkeypatch):
    """Queued is already cancellable: the operator submitted it and pressed stop."""
    child = _child(monkeypatch)
    session = _StubSession()
    child._session = session
    lines = "\n".join([
        json.dumps({"cmd": "task", "task_id": "t1", "instruction": "向前飞两米"}),
        json.dumps({"cmd": "stop"}),
        "",
    ])
    monkeypatch.setattr(sys, "stdin", io.StringIO(lines))
    child._reader()
    while not child._commands.empty():
        command = child._commands.get_nowait()
        if command["cmd"] == "task":
            child._run_task(command)
    assert session.seen == [("t1", True)], session.seen


def test_a_stop_that_lands_mid_task_cancels_that_task(monkeypatch):
    """The reader thread is the stop's path, so drive that thread for real."""
    from planner.domain.result import MissionResult

    child = _child(monkeypatch)
    child._session = _StubSession()
    during = {}

    def run(task, stop_event=None, log=None):
        during["before_stop"] = stop_event.is_set()
        monkeypatch.setattr(sys, "stdin", io.StringIO(json.dumps({"cmd": "stop"}) + "\n"))
        child._reader()                     # what the stdin thread does on a stop
        during["after_stop"] = stop_event.is_set()
        return MissionResult(success=False, stages=(), error="cancelled: task was cancelled")

    child._session.run = run
    child._run_task({"cmd": "task", "task_id": "t1", "instruction": "向前飞八米"})
    assert during == {"before_stop": False, "after_stop": True}, during
    assert child._stop_requested is False, "the cancellation must not outlive its task"
