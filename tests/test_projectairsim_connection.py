"""Simulator-time advance hooks on the ProjectAirSim connection."""

import threading
import time

import pytest

from planner.errors import SimulatorConnectionError
from planner.simulators.projectairsim.connection import ProjectAirSimConnection


class FakeWorld:
    def __init__(self):
        self.now_ns = 0

    def continue_for_sim_time(self, nanoseconds, wait_until_complete=True):
        self.now_ns += nanoseconds
        return self.now_ns

    def get_sim_time(self):
        return self.now_ns


def make_connection():
    connection = ProjectAirSimConnection.__new__(ProjectAirSimConnection)
    connection._lock = threading.RLock()
    connection._hooks_lock = threading.Lock()
    connection._advance_hooks = []
    connection.client = None
    connection.world = FakeWorld()
    connection._owns_active = False
    return connection


def test_advance_hook_receives_the_new_simulator_time():
    connection = make_connection()
    seen = []
    connection.add_advance_hook(seen.append)

    assert connection.advance_for_sim_time(0.05) == 50_000_000
    assert seen == [50_000_000]


def test_advance_hook_failure_does_not_break_the_advance(caplog):
    connection = make_connection()
    calls = []

    def failing(sim_ns):
        calls.append(sim_ns)
        raise RuntimeError("boom")

    connection.add_advance_hook(failing)

    assert connection.advance_for_sim_time(1.0) == 1_000_000_000
    assert calls == [1_000_000_000]
    assert "advance hook failed" in caplog.text


def test_removed_hook_no_longer_runs():
    connection = make_connection()
    seen = []
    hook = seen.append
    connection.add_advance_hook(hook)

    connection.remove_advance_hook(hook)
    connection.advance_for_sim_time(0.1)

    assert seen == []


class RunningWorld(FakeWorld):
    """A world whose clock moves with wall time, like a resumed engine."""

    def __init__(self):
        super().__init__()
        self._started = time.monotonic()
        self.continue_calls = 0
        self.resume_calls = 0
        self.pause_calls = 0

    def continue_for_sim_time(self, nanoseconds, wait_until_complete=True):
        self.continue_calls += 1
        return super().continue_for_sim_time(nanoseconds, wait_until_complete)

    def get_sim_time(self):
        return int((time.monotonic() - self._started) * 1e9)

    def resume(self):
        self.resume_calls += 1
        return "resumed"

    def pause(self):
        self.pause_calls += 1
        return "paused"


class StalledWorld(RunningWorld):
    """A world that ignores resume and keeps reporting one instant."""

    def get_sim_time(self):
        return 0


def test_free_running_advance_waits_for_the_clock_instead_of_stepping():
    connection = make_connection()
    world = connection.world = RunningWorld()
    seen = []
    connection.add_advance_hook(seen.append)
    assert connection.resume_clock() is True

    before = world.get_sim_time()
    returned = connection.advance_for_sim_time(0.05)

    assert world.continue_calls == 0
    assert returned >= before + 50_000_000
    assert seen == [returned]


def test_stalled_clock_fails_instead_of_waiting_forever(monkeypatch):
    monkeypatch.setattr(
        "planner.simulators.projectairsim.connection._CLOCK_STALL_TIMEOUT_S", 0.05
    )
    connection = make_connection()
    connection.world = StalledWorld()
    connection._free_running = True

    with pytest.raises(SimulatorConnectionError, match="did not advance"):
        connection.advance_for_sim_time(0.05)


def test_resume_clock_falls_back_when_the_engine_ignores_it(caplog):
    connection = make_connection()
    world = connection.world = StalledWorld()

    assert connection.resume_clock(verify_s=0.05) is False
    assert world.pause_calls == 1

    # The fallback keeps the stepped behaviour, one request per period.
    assert connection.advance_for_sim_time(0.05) == 50_000_000
    assert world.continue_calls == 1
