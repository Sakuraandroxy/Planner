"""Background threads that outlive their owner must say so, and stop blocking calls.

Three runtime helpers used to join with ``timeout=1.0`` and then walk away silently,
one cleanup thread joined with no timeout at all, and ``ModelCallService`` kept its
slot busy forever after a timed-out call -- so a wedged model endpoint turned into a
permanently refused stage. The cases here pin the convergence: bounded waits, a
logged straggler, and a superseded result that is thrown away instead of resurrecting
a call the caller already saw time out.
"""

import logging
import threading
import time

import pytest

from planner.errors import ExecutionError, ModelCallTimeout
from planner.runtime.background_operation import BackgroundOperation
from planner.runtime.model_calls import ModelCallService
from planner.runtime.resource_cleanup import stop_thread
from planner.runtime.window.worker import SlowPlannerWorker


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_stop_thread_names_a_thread_that_will_not_end(caplog):
    release = threading.Event()
    thread = threading.Thread(target=release.wait, daemon=True)
    thread.start()
    try:
        with caplog.at_level(logging.WARNING):
            assert stop_thread(thread, "stuck-thread", timeout_s=0.05) is True
        assert "stuck-thread did not stop within 0.05s" in caplog.text
    finally:
        release.set()
        thread.join(timeout=1.0)
    assert stop_thread(thread, "stuck-thread") is False


def test_a_slow_planner_close_is_bounded_and_logged(caplog):
    started = threading.Event()
    release = threading.Event()

    def model_call():
        started.set()
        release.wait(10.0)
        return object()

    worker = SlowPlannerWorker()
    worker.submit(model_call)
    assert started.wait(5.0)
    before = time.monotonic()
    with caplog.at_level(logging.WARNING):
        worker.close()
    assert time.monotonic() - before < 5.0
    assert "planner-slow-model did not stop" in caplog.text
    with pytest.raises(ExecutionError, match="closed"):
        worker.require_available()
    release.set()


def test_a_background_operation_that_never_returns_does_not_hold_the_close(caplog):
    started = threading.Event()
    release = threading.Event()
    closed = []

    def operation():
        started.set()
        release.wait(10.0)

    job = BackgroundOperation()
    job.start(operation, 30.0)
    assert started.wait(5.0)
    before = time.monotonic()
    with caplog.at_level(logging.WARNING):
        job.close(close_idle=lambda: closed.append(True))
    assert time.monotonic() - before < 5.0
    assert "planner-data-operation did not stop" in caplog.text
    assert closed == []                      # the transport outlives its pending RPC
    release.set()


def test_a_superseded_model_call_lets_the_next_one_run(caplog):
    """A hung endpoint used to leave ``_busy`` true for good."""
    release = threading.Event()

    def hung():
        release.wait(10.0)
        return "late"

    service = ModelCallService()
    with pytest.raises(ModelCallTimeout):
        service.call(hung, 0.05, checkpoint=lambda: None)
    assert service.has_pending_call               # still quarantined, honestly

    # Past its deadline the slot is free, so the stage can make progress again.
    service.require_available()
    assert service.call(lambda: "fresh", 2.0, checkpoint=lambda: None) == "fresh"

    with caplog.at_level(logging.WARNING):
        release.set()
        assert _wait_for(lambda: "superseded call" in caplog.text), caplog.text
    service.close()


def test_close_refuses_further_calls():
    service = ModelCallService()
    service.close()
    with pytest.raises(ExecutionError, match="closed"):
        service.require_available()
    with pytest.raises(ExecutionError, match="closed"):
        service.call(lambda: 1, 1.0, checkpoint=lambda: None)
