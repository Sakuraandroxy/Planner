"""Simulator-time advance hooks on the ProjectAirSim connection."""

import threading

from planner.adapters.projectairsim.connection import ProjectAirSimConnection


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
