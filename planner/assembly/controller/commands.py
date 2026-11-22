"""Command queue, worker delivery, connect/teardown and the supervisor loop.

HTTP callers only enqueue here; every worker call happens on the supervisor
thread. A worker failure or a connect error transitions back to ``connecting``
and retries after ``reconnect_interval_s``.
"""

from __future__ import annotations

import queue
import threading
import time

from ...domain.server import (
    BackendUnavailable,
    ServerState,
    TaskSubmission,
    WorkerEventKind,
)
from ...domain.view import default_view_for


class CommandChannel:
    def __init__(self, controller):
        self._c = controller

    def start(self) -> None:
        c = self._c
        if c._thread is not None:
            return
        if not c._session_started:
            c.world.stop_leftover_simulator()
            c.lifecycle.log(
                "No scene is loaded. Choose a map in the console to start "
                "the simulator."
            )
        c._thread = threading.Thread(
            target=self.supervise, name="planner-supervisor", daemon=True
        )
        c._thread.start()

    def submit(
        self, instruction: str, vehicle: str | None = None
    ) -> TaskSubmission:
        c = self._c
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("instruction must be non-empty text")
        requested = c._vehicle if vehicle is None else str(vehicle).strip()
        if requested not in c._vehicles:
            raise ValueError(
                f"unknown vehicle '{requested}'; expected one of "
                f"{list(c._vehicles)}"
            )
        with c._lock:
            if c._closing.is_set() or c._state is not ServerState.IDLE:
                raise BackendUnavailable(c._state)
            c._sequence += 1
            task = TaskSubmission(
                f"task_{c._sequence}",
                instruction.strip(),
                requested,
            )
            c._active = task.task_id
            c._instruction = task.instruction
            c._vehicle = requested
            c._state = ServerState.RUNNING
            c._commands.put(("task", c._generation, task))
        return task

    def stop(self) -> ServerState:
        c = self._c
        with c._lock:
            if c._state is ServerState.RUNNING:
                c._state = ServerState.STOPPING
                c._commands.put(("stop", c._generation, None))
            return c._state

    def home(self, vehicle: str | None = None) -> str:
        """Teleport a vehicle back to its spawn pose; only while idle."""
        c = self._c
        with c._lock:
            if (
                c._closing.is_set()
                or c._state is not ServerState.IDLE
                or c._worker is None
                or not c._worker_ready
            ):
                raise BackendUnavailable(c._state)
            requested = c._vehicle if vehicle is None else str(vehicle).strip()
            if requested not in c._vehicles:
                raise ValueError(
                    f"unknown vehicle '{requested}'; expected one of "
                    f"{list(c._vehicles)}"
                )
            c._commands.put(("home", c._generation, requested))
            return requested

    def activate(self, vehicle: str) -> str:
        """Console switch: park the other vehicle, home + show this one.

        The status updates optimistically; a failed ACTIVE event restores the
        previous selection and view.
        """
        c = self._c
        with c._lock:
            if (
                c._closing.is_set()
                or c._state is not ServerState.IDLE
                or c._worker is None
                or not c._worker_ready
            ):
                raise BackendUnavailable(c._state)
            requested = str(vehicle).strip()
            if requested not in c._vehicles:
                raise ValueError(
                    f"unknown vehicle '{requested}'; expected one of "
                    f"{list(c._vehicles)}"
                )
            c._pending_activation = {
                "vehicle": c._vehicle,
                "view_mode": c._view_mode,
            }
            c._vehicle = requested
            c._view_mode = default_view_for(requested)
            c._view_pending = True
            c._commands.put(("activate", c._generation, requested))
        return requested

    def deliver(self, command) -> None:
        c = self._c
        kind, generation, value = command
        with c._lock:
            if generation != c._generation:
                return
            state, active = c._state, c._active
            worker = c._worker
        if kind == "world":
            c.world.switch_world(value)
            return
        if kind == "models":
            c.models.switch_models()
            return
        if kind == "profile":
            c.profile_switcher.switch_profile()
            return
        if kind == "stop_world":
            c.world.stop_world_command()
            return
        if worker is None:
            return
        if (
            kind == "task"
            and state is ServerState.RUNNING
            and value.task_id == active
        ):
            c.lifecycle.log(
                f"Task {value.task_id} accepted: {value.instruction}"
            )
            worker.submit(value)
        elif kind == "stop" and state is ServerState.STOPPING:
            c.lifecycle.log(
                "Stopping the task; motion aborts between setpoints."
            )
            worker.stop()
        elif kind == "home" and state is ServerState.IDLE:
            c.lifecycle.log(f"Returning the {value} to its start position.")
            worker.home(value)
        elif kind == "activate" and state is ServerState.IDLE:
            c.lifecycle.log(
                f"Activating the {value}: parking the other vehicle and "
                "returning it home."
            )
            worker.activate(value)
        elif kind == "view" and state is ServerState.IDLE:
            c.lifecycle.log(f"Applying streaming view: {value}.")
            worker.set_view_mode(value)
            c._applied_view = value
        elif kind == "weather" and state is ServerState.IDLE:
            worker.set_weather(value)
            c._weather_applied = True

    def connect(self) -> None:
        c = self._c
        try:
            worker = c.worker_factory(c._selection)
        except Exception as error:
            c.lifecycle.log(
                f"Worker construction failed: {type(error).__name__}: {error}",
                "ERROR",
            )
            c._next_start = c.clock() + c.settings.reconnect_interval_s
            return
        with c._lock:
            c._generation += 1
            c._worker = worker
            c._worker_ready = False
            c._started_at = c.clock()
            c._state = ServerState.CONNECTING
        worker.start()

    def failed(self, reason: str) -> None:
        c = self._c
        c.lifecycle.log(reason, "ERROR")
        worker, c._worker = c._worker, None
        if worker is not None:
            try:
                worker.terminate()
            except Exception:
                pass
        with c._lock:
            c._generation += 1
            c._active = None
            c._worker_ready = False
            c._state = ServerState.CONNECTING
            c._instruction = ""
            c._weather_applied = False
            c._pending_activation = None
            c._view_pending = False
            # A switch that died halfway must not leave the console reporting a
            # switch forever: the retry rebuilds with whatever is already selected.
            c._models_switching = False
            c._profile_switching = False
        c._next_start = c.clock() + c.settings.reconnect_interval_s

    def close_worker(self, *, skip_landing: bool = False) -> bool:
        """Shut the worker down and wait for CLOSED before a restart."""
        c = self._c
        with c._lock:
            worker = c._worker
        if worker is None:
            with c._lock:
                c._worker_ready = False
            c.lifecycle.reset_session_state()
            return True
        try:
            worker.shutdown(skip_landing=skip_landing)
        except Exception as error:
            c.lifecycle.log(f"Worker shutdown failed: {error}", "ERROR")
            try:
                worker.terminate()
            except Exception:
                pass
        if not self.wait_closed(worker):
            return False
        with c._lock:
            if c._worker is worker:
                c._worker = None
            c._worker_ready = False
        c.lifecycle.reset_session_state()
        return True

    def wait_closed(self, worker) -> bool:
        c = self._c
        deadline = c.clock() + c._worker_shutdown_timeout_s
        closed = False
        while True:
            if self.drain(worker):
                closed = True
            if closed:
                return True
            if c.clock() >= deadline:
                return False
            time.sleep(0.05)

    def drain(self, worker) -> bool:
        """Consume available events; return True when CLOSED was seen."""
        c = self._c
        closed = False
        for _ in range(100):
            event = worker.poll()
            if event is None:
                break
            if event.kind is WorkerEventKind.LOG:
                c.lifecycle.log(event.message, event.level)
            elif event.kind is WorkerEventKind.CLOSED:
                closed = True
            elif event.kind is WorkerEventKind.WEATHER:
                with c._lock:
                    if event.payload is not None:
                        c._weather_catalog = event.payload
        return closed

    def supervise(self) -> None:
        c = self._c
        try:
            while not c._closing.is_set():
                try:
                    command = c._commands.get(timeout=0.05)
                except queue.Empty:
                    command = None
                try:
                    if command is not None:
                        self.deliver(command)
                    if (
                        c._pending_world is not None
                        and c.clock() >= c._world_retry_at
                    ):
                        c.world.switch_world(c._pending_world)
                    if (
                        c._session_started
                        and c._worker is None
                        and c.clock() >= c._next_start
                    ):
                        self.connect()
                    c.lifecycle.on_event()
                    if (
                        c._worker is not None
                        and not c._worker_ready
                        and c.clock() - c._started_at
                        >= c.settings.startup_timeout_s
                    ):
                        self.failed(
                            "Simulator session setup timed out. Retrying."
                        )
                except Exception as error:
                    self.failed(
                        f"Backend lifecycle error: {type(error).__name__}: "
                        f"{error}"
                    )
        finally:
            self.shutdown_worker()

    def shutdown_worker(self) -> None:
        c = self._c
        worker, c._worker = c._worker, None
        with c._lock:
            c._worker_ready = False
        if worker is None:
            return
        try:
            worker.shutdown()
        except Exception as error:
            c.lifecycle.log(f"Worker shutdown failed: {error}", "ERROR")
            try:
                worker.terminate()
            except Exception:
                pass
        if not self.drain(worker):
            c.lifecycle.log(
                "Worker did not confirm shutdown within the timeout.", "ERROR"
            )

    def close(self) -> None:
        c = self._c
        with c._lock:
            if c._closing.is_set():
                return
            c._pending_activation = None
            c._view_pending = False
            c._state = ServerState.STOPPING
            c._closing.set()
        try:
            if c._thread is not None:
                c._thread.join(
                    timeout=c.settings.startup_timeout_s + 8.0
                )
                if c._thread.is_alive():
                    raise RuntimeError(
                        "planner supervisor did not finish shutdown"
                    )
        finally:
            # Even when the monitor thread is wedged: at this point nothing is left to
            # interrupt, and leaving a UE process holding the GPU is the worse failure.
            c.world.stop_simulator()
