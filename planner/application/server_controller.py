"""One backend lifecycle owner; HTTP callers only submit commands and read data."""
import logging
import queue
import threading
import time
from typing import Callable

from planner.domain.server import (BackendUnavailable, ServerState, ServerStatus, TaskSubmission,
                                   WorkerEventKind, WorkerMode)
from planner.ports.backend_runtime import BackendWorker, ConnectionMonitor
from planner.services.log_buffer import LogBuffer

logger = logging.getLogger(__name__)


class ServerController:
    def __init__(self, worker_factory: Callable[[WorkerMode], BackendWorker],
                 monitor_factory: Callable[[], ConnectionMonitor], settings, *,
                 startup_timeout_s=65., clock=time.monotonic):
        self.worker_factory, self.monitor_factory, self.settings = worker_factory, monitor_factory, settings
        self.startup_timeout_s, self.clock = startup_timeout_s, clock
        self.logs = LogBuffer(settings.log_buffer_size)
        self._state = ServerState.CONNECTING
        self._lock = threading.Lock()
        self._commands = queue.Queue()
        self._closing = threading.Event()
        self._thread = self._heartbeat = None
        self._worker = None
        self._had_worker = False
        self._worker_ready = False
        self._generation = 0
        self._sequence = 0
        self._active = None
        self._instruction = ""
        self._next_start = 0.
        self._mode = WorkerMode.PREPARE
        self._started_at = 0.

    @property
    def state(self) -> ServerState:
        with self._lock:
            return self._state

    @property
    def status(self) -> ServerStatus:
        with self._lock:
            return ServerStatus(self._state, self._instruction)

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="planner-supervisor", daemon=True)
        self._heartbeat = threading.Thread(target=self._monitor, name="planner-heartbeat", daemon=True)
        self._thread.start()
        self._heartbeat.start()

    def submit(self, instruction: str) -> TaskSubmission:
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("instruction must be non-empty text")
        with self._lock:
            if self._closing.is_set() or self._state is not ServerState.IDLE:
                raise BackendUnavailable(self._state)
            self._sequence += 1
            task = TaskSubmission(f"task_{self._sequence}", instruction.strip())
            self._active = task.task_id
            self._instruction = task.instruction
            self._state = ServerState.RUNNING
            self._commands.put(("task", self._generation, task))
        return task

    def stop(self) -> ServerState:
        with self._lock:
            if self._state is ServerState.RUNNING:
                self._state = ServerState.STOPPING
                self._commands.put(("stop", self._generation, None))
            return self._state

    def _log(self, text, level="INFO"):
        self.logs.append(text, level)
        logger.log(getattr(logging, level, logging.INFO), "%s", text)

    def _monitor(self):
        monitor = None
        try:
            while not self._closing.is_set():
                with self._lock:
                    generation = self._generation
                try:
                    if monitor is None:
                        monitor = self.monitor_factory()
                    healthy = monitor.check()
                except Exception:
                    healthy = False
                    if monitor is not None:
                        monitor.close()
                    monitor = None
                self._commands.put(("heartbeat", generation, healthy))
                self._closing.wait(self.settings.heartbeat_interval_s)
        finally:
            if monitor is not None:
                monitor.close()

    def _replace(self, mode, *, delay=False):
        # Keep the old handle until terminate confirms exit. Never overlap owners.
        if self._worker is not None:
            self._worker.terminate()
            self._worker = None
        with self._lock:
            self._generation += 1
            self._active = None
            self._worker_ready = False
        self._mode = mode
        self._next_start = self.clock() + (self.settings.reconnect_interval_s if delay else 0.)

    def _reconnect(self, reason):
        with self._lock:
            already_disconnected = self._state is ServerState.CONNECTING and self._worker is None
            self._state = ServerState.CONNECTING
            self._instruction = ""
        if already_disconnected:
            return
        self._log(reason, "ERROR")
        self._replace(WorkerMode.PREPARE, delay=True)

    def _retry_preparation(self, reason):
        # A hold failure is not evidence that the map changed. Never take off
        # merely to recover a failed stop or a failed runtime assembly.
        with self._lock:
            mode = WorkerMode.HOLD if self._worker_ready else self._mode
            self._state = ServerState.STOPPING if mode is WorkerMode.HOLD else ServerState.CONNECTING
            if self._state is ServerState.CONNECTING:
                self._instruction = ""
        self._log(reason, "ERROR")
        self._replace(mode, delay=True)

    def _command(self, command):
        kind, generation, value = command
        with self._lock:
            if generation != self._generation:
                return
            state, active = self._state, self._active
        if kind == "task" and state is ServerState.RUNNING and value.task_id == active:
            self._log(f"Task {value.task_id} accepted: {value.instruction}")
            self._worker.submit(value)
        elif kind == "stop" and state is ServerState.STOPPING:
            self._log("Stopping task. Discarding the old execution process and trajectory queue.")
            self._replace(WorkerMode.HOLD)
        elif kind == "heartbeat" and not value:
            self._reconnect("AirSim heartbeat failed. The old task will not resume.")

    def _events(self):
        for _ in range(100):
            worker = self._worker
            if worker is None:
                return
            event = worker.poll()
            if event is None:
                return
            if event.kind is WorkerEventKind.LOG:
                self._log(event.message, event.level)
            elif event.kind is WorkerEventKind.READY:
                with self._lock:
                    # A stop may have been accepted after the previous loop tick.
                    if self._state is ServerState.STOPPING and self._mode is not WorkerMode.HOLD:
                        continue
                    self._worker_ready = True
                    self._state = ServerState.IDLE
                    self._instruction = ""
                self._log("Planner ready. State: idle.")
            elif event.kind is WorkerEventKind.FINISHED:
                with self._lock:
                    if self._state is not ServerState.RUNNING or event.task_id != self._active:
                        continue
                    self._active = None
                    self._state = ServerState.IDLE if event.success else ServerState.STOPPING
                    if event.success:
                        self._instruction = ""
                if not event.success:
                    self._log("Task failed. Replacing its runtime before accepting another task.", "WARNING")
                    self._replace(WorkerMode.HOLD)
            elif event.kind is WorkerEventKind.ERROR:
                self._retry_preparation(event.message or "Planner preparation failed.")
                return
            elif event.kind is WorkerEventKind.DISCONNECTED:
                self._reconnect(event.message or "Planner execution process disconnected.")
                return
            elif event.kind is WorkerEventKind.EXITED:
                self._retry_preparation(event.message or "Planner execution process exited.")
                return

    def _run(self):
        try:
            while not self._closing.is_set():
                try:
                    command = self._commands.get(timeout=.05)
                except queue.Empty:
                    command = None
                try:
                    if command is not None:
                        self._command(command)
                    if self._worker is None and self.clock() >= self._next_start:
                        self._worker = self.worker_factory(self._mode)
                        self._had_worker = True
                        self._started_at = self.clock()
                    self._events()
                    if (self._worker is not None and not self._worker_ready and
                            self.clock()-self._started_at >= self.startup_timeout_s):
                        self._retry_preparation("Planner preparation timed out. Retrying the same preparation mode.")
                except Exception as exc:
                    self._retry_preparation(f"Backend lifecycle error: {exc}")
        finally:
            self._shutdown_worker()

    def _shutdown_worker(self):
        if not self._had_worker:
            return
        # Always cancel any server-side motion through a new sole owner after
        # killing a blocked parser or native AirSim command in the old process.
        if self._worker is not None:
            self._worker.terminate()
            self._worker = None
        cleanup = self.worker_factory(WorkerMode.CLEANUP)
        deadline = self.clock() + self.startup_timeout_s
        stopped = False
        try:
            while self.clock() < deadline:
                event = cleanup.poll()
                if event is not None:
                    if event.kind is WorkerEventKind.CLOSED:
                        stopped = True
                        break
                    if event.message:
                        self._log(event.message, event.level)
                    if event.kind in (WorkerEventKind.ERROR, WorkerEventKind.DISCONNECTED, WorkerEventKind.EXITED):
                        break
                time.sleep(.02)
            if not stopped:
                self._log("Server exit could not confirm the vehicle stop.", "ERROR")
        finally:
            cleanup.shutdown() if stopped else cleanup.terminate()

    def close(self):
        with self._lock:
            self._state = ServerState.STOPPING
            self._closing.set()
        if self._heartbeat is not None:
            self._heartbeat.join(timeout=self.settings.heartbeat_timeout_s + 2)
        if self._thread is not None:
            self._thread.join(timeout=self.startup_timeout_s + 8)
            if self._thread.is_alive():
                raise RuntimeError("Planner supervisor did not finish shutdown")
