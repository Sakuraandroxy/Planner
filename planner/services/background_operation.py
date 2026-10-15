"""One bounded data-only operation, with late-result quarantine and explicit close."""
from concurrent.futures import Future
import threading
import time
import math

from planner.errors import ExecutionError, ModelCallTimeout


class BackgroundOperation:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.future = None
        self.deadline = 0.0
        self._thread = None
        self._busy = False
        self._closed = False
        self._lock = threading.Lock()
        self._owner = threading.get_ident()

    @property
    def busy(self):
        with self._lock:
            return self._busy

    def require_available(self):
        if threading.get_ident() != self._owner:
            raise ExecutionError("only control owner may manage background operations")
        if self._closed or self.busy or self.future is not None:
            raise ExecutionError("background operation is unavailable or quarantined")

    def start(self, operation, timeout_s):
        self.require_available()
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ExecutionError("background deadline exhausted")
        future = Future()
        self.future, self.deadline = future, self.clock()+timeout_s
        with self._lock:
            self._busy = True

        def run():
            try:
                value, error = operation(), None
            except BaseException as exc:
                value, error = None, exc
            finished = self.clock()
            with self._lock:
                self._busy = False
                if error is None:
                    future.set_result((value, finished))
                else:
                    future.set_exception(error)

        self._thread = threading.Thread(target=run, name="planner-data-operation", daemon=True)
        try:
            self._thread.start()
        except BaseException:
            with self._lock:
                self._busy = False
            self.future = None
            raise

    def poll(self):
        if threading.get_ident() != self._owner:
            raise ExecutionError("only control owner may collect background operations")
        if self.future is None:
            return None
        if not self.future.done():
            if self.clock() >= self.deadline:
                self.future = None
                raise ModelCallTimeout("background operation timed out; late result quarantined")
            return None
        future, self.future = self.future, None
        value, finished = future.result()
        if finished >= self.deadline:
            raise ModelCallTimeout("late background result discarded")
        return value

    def discard(self):
        self.future = None

    def close(self, close_idle=None):
        self._closed = True
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if close_idle is not None:
            if not self.busy:
                close_idle()
            elif self._thread is not None:
                # A read-only transport is closed only after its pending RPC
                # returns. This helper cannot send vehicle commands.
                thread = self._thread
                def finish():
                    thread.join()
                    close_idle()
                threading.Thread(target=finish, name="planner-read-cleanup", daemon=True).start()
