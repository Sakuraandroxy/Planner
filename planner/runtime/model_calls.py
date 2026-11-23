"""Bounded single-flight model calls; only the caller runs safety checkpoints.

Operations must contain model I/O only, never vehicle/queue access. A running
HTTP request cannot be forcibly killed: on timeout its future is detached, and a
new call may take over once the previous one is past its deadline. The late
result is discarded and logged when it lands.
"""
import math
import logging
import threading
import time
from collections.abc import Callable
from concurrent.futures import Future
from typing import TypeVar

from planner.errors import ExecutionError, ModelCallTimeout

from .resource_cleanup import stop_thread

logger = logging.getLogger(__name__)

T = TypeVar("T")


class ModelCallService:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._owner = threading.get_ident()
        self._lock = threading.Lock()
        self._busy = False
        self._closed = False
        self._thread: threading.Thread | None = None
        #: Wall-clock deadline of the call currently holding the slot.
        self._deadline = 0.0
        #: Bumped by every call; a result carrying an older one is discarded.
        self._generation = 0

    def require_available(self) -> None:
        if threading.get_ident() != self._owner:
            raise ExecutionError("only the control owner may collect model calls")
        with self._lock:
            if self._closed:
                raise ExecutionError("model caller is closed")
            if self._busy and self.clock() < self._deadline:
                raise ExecutionError(
                    "a previous model request is still running"
                )
            # Past its deadline the previous call has already been reported as a
            # timeout, so the slot is free: the next call takes it over and the
            # late result is thrown away when the stuck thread finally returns.

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
                superseded = generation != self._generation
                if not superseded:
                    self._busy = False
                    if error is None:
                        future.set_result((result, finished))
                    else:
                        future.set_exception(error)
            if superseded:
                logger.warning(
                    "model result from a superseded call arrived after its "
                    "deadline; discarded"
                )

        with self._lock:
            self._generation += 1
            generation = self._generation
            self._busy = True
            self._deadline = deadline
            self._thread = threading.Thread(target=run, name="planner-model-call", daemon=True)
            try:
                self._thread.start()
            except BaseException:
                self._busy = False
                self._deadline = 0.0
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
        stop_thread(thread, "planner-model-call")