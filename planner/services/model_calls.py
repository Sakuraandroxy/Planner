"""Bounded single-flight model calls; only the caller runs safety checkpoints.

Operations must contain model I/O only, never vehicle/queue access. A running
HTTP request cannot be forcibly killed: on timeout its future is detached,
and another call is refused until that operation actually returns.
"""
from concurrent.futures import Future
import math
import threading
import time
from typing import Callable, TypeVar

from planner.errors import ExecutionError, ModelCallTimeout

T = TypeVar("T")


class ModelCallService:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._owner = threading.get_ident()
        self._lock = threading.Lock()
        self._busy = False
        self._closed = False
        self._thread: threading.Thread | None = None

    def require_available(self) -> None:
        if threading.get_ident() != self._owner:
            raise ExecutionError("only the control owner may collect model calls")
        with self._lock:
            if self._closed or self._busy:
                raise ExecutionError("model caller is closed or a previous request is still running")

    @property
    def has_pending_call(self) -> bool:
        """Read-only quarantine state, including calls detached after timeout."""
        with self._lock:
            return self._busy

    def call(self, operation: Callable[[], T], timeout_s: float,
             checkpoint: Callable[[], None]) -> T:
        self.require_available()
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ExecutionError("model call timeout must be finite and positive")
        checkpoint()
        deadline = self.clock() + timeout_s
        future: Future[tuple[T, float]] = Future()

        def run():
            result, error = None, None
            try:
                result = operation()
            except BaseException as exc:
                error = exc
            finished = self.clock()
            with self._lock:
                self._busy = False
                if error is None:
                    future.set_result((result, finished))
                else:
                    future.set_exception(error)

        with self._lock:
            self._busy = True
            self._thread = threading.Thread(target=run, name="planner-model-call", daemon=True)
            try:
                self._thread.start()
            except BaseException:
                self._busy = False
                self._thread = None
                raise
        while True:
            checkpoint()
            now = self.clock()
            if now >= deadline:
                raise ModelCallTimeout("model call wall-clock timeout; late result quarantined")
            if future.done():
                result, finished = future.result()
                if finished >= deadline:
                    raise ModelCallTimeout("model response arrived after its deadline")
                return result
            threading.Event().wait(min(0.02, deadline - now))

    def close(self) -> None:
        if threading.get_ident() != self._owner:
            raise ExecutionError("only the control owner may close model calls")
        with self._lock:
            self._closed = True
            thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
