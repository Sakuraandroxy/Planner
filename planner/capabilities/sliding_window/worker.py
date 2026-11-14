"""One lazy model-only worker. No callbacks, queue mutation or flight I/O."""
from concurrent.futures import Future
import threading
from typing import Callable

from planner.domain.sliding_window import PlanningOutput
from planner.errors import ExecutionError


class SlowPlannerWorker:
    def __init__(self):
        self._condition = threading.Condition()
        self._job: tuple[Future, Callable[[], PlanningOutput]] | None = None
        self._busy = False
        self._closed = False
        self._thread: threading.Thread | None = None

    def require_available(self) -> None:
        with self._condition:
            if self._closed or self._busy:
                raise ExecutionError("slow planner is closed or a previous request is still running")

    @property
    def has_pending_call(self) -> bool:
        with self._condition:
            return self._busy

    def submit(self, operation: Callable[[], PlanningOutput]) -> Future:
        with self._condition:
            self.require_available()
            future = Future()
            self._busy = True
            self._job = (future, operation)
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="planner-slow-model", daemon=True)
                self._thread.start()
            self._condition.notify_all()
            return future

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._job is not None)
                if self._job is None:
                    return
                future, operation = self._job
                self._job = None
            result, error = None, None
            if future.set_running_or_notify_cancel():
                try:
                    result = operation()
                except BaseException as exc:
                    error = exc
            # Complete the future while holding the submission lock: a caller
            # observing done() may safely submit the next request immediately.
            with self._condition:
                self._busy = False
                if not future.cancelled():
                    if error is not None:
                        future.set_exception(error)
                    else:
                        future.set_result(result)
                self._condition.notify_all()
                if self._closed:
                    return

    def close(self) -> None:
        with self._condition:
            self._closed = True
            if self._job is not None:
                self._job[0].cancel()
            self._condition.notify_all()
        if self._thread is not None:
            # HTTP already in progress cannot be killed with Future.cancel().
            # It has a request timeout, is isolated, and cannot mutate flight
            # state. A daemon avoids hanging process exit on a broken server.
            self._thread.join(timeout=1.0)
