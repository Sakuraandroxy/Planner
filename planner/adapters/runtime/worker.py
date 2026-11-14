"""In-process worker thread that owns the simulator session.

The HTTP layer never touches the simulator. The worker builds the session on
its own thread, executes one task at a time and publishes lifecycle events
through a queue polled by the server controller.
"""

from __future__ import annotations

import queue
import threading
from typing import Callable

from ...application.reporting import mission_report
from ...domain.server import TaskSubmission, WorkerEvent, WorkerEventKind


class ThreadedMissionWorker:
    def __init__(
        self,
        session_factory: Callable[[Callable[[str], None]], object],
        *,
        join_timeout_s: float = 10.0,
    ):
        self._session_factory = session_factory
        self._join_timeout_s = float(join_timeout_s)
        self._commands: queue.Queue = queue.Queue()
        self._events: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._stop_requested = False
        self._closing = threading.Event()
        # Set when the world itself is about to be replaced (map or model
        # switch): the drone does not need to land in a scene being destroyed.
        self._skip_landing = False

    # -- BackendWorker port ------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="vln-worker", daemon=True
        )
        self._thread.start()

    def submit(self, task: TaskSubmission) -> None:
        self._commands.put(("task", task))

    def stop(self) -> None:
        with self._lock:
            self._stop_requested = True
            self._stop_event.set()

    def home(self, vehicle: str | None = None) -> None:
        """Ask the idle session to teleport a vehicle back to its spawn."""
        if self._thread is not None:
            self._commands.put(("home", vehicle))

    def activate(self, vehicle: str) -> None:
        """Park the other vehicle, home the requested one, show its front view."""
        if self._thread is not None:
            self._commands.put(("activate", vehicle))

    def set_view_mode(self, mode: str) -> None:
        if self._thread is not None:
            self._commands.put(("view", mode))

    def set_weather(self, fields: dict[str, float]) -> None:
        if self._thread is not None:
            self._commands.put(("weather", dict(fields)))

    def poll(self) -> WorkerEvent | None:
        try:
            return self._events.get_nowait()
        except queue.Empty:
            return None

    def shutdown(self, *, skip_landing: bool = False) -> None:
        self._skip_landing = bool(skip_landing)
        self._closing.set()
        with self._lock:
            self._stop_event.set()
        self._commands.put(("shutdown", None))
        self._join()

    def terminate(self) -> None:
        self.shutdown()

    # -- internals ---------------------------------------------------------
    def _join(self) -> None:
        if self._thread is not None:
            self._thread.join(timeout=self._join_timeout_s)

    def _emit(self, event: WorkerEvent) -> None:
        self._events.put(event)

    def _log(self, message: str, level: str = "INFO") -> None:
        self._emit(WorkerEvent(WorkerEventKind.LOG, message=message, level=level))

    def _run(self) -> None:
        session = None
        try:
            self._log("Connecting to ProjectAirSim for the mission worker.")
            try:
                session = self._session_factory(self._log)
            except Exception as error:
                self._emit(
                    WorkerEvent(
                        WorkerEventKind.ERROR,
                        message=(
                            "ProjectAirSim session setup failed: "
                            f"{type(error).__name__}: {error}"
                        ),
                    )
                )
                return
            self._emit(WorkerEvent(WorkerEventKind.READY))
            catalog = getattr(session, "weather_catalog", None)
            if catalog is not None:
                try:
                    self._emit(
                        WorkerEvent(WorkerEventKind.WEATHER, payload=catalog())
                    )
                except Exception as error:
                    self._log(
                        "weather catalog unavailable: "
                        f"{type(error).__name__}: {error}",
                        "WARNING",
                    )
            self._loop(session)
        finally:
            if session is not None:
                try:
                    session.close(skip_landing=self._skip_landing)
                except Exception as error:
                    self._log(
                        f"ProjectAirSim session close failed: {error}", "WARNING"
                    )
            self._emit(WorkerEvent(WorkerEventKind.CLOSED))

    def _loop(self, session) -> None:
        idle_tick = getattr(session, "idle_tick", None)
        while not self._closing.is_set():
            try:
                kind, value = self._commands.get(timeout=0.05)
            except queue.Empty:
                if idle_tick is not None:
                    try:
                        idle_tick()
                    except Exception as error:
                        self._emit(
                            WorkerEvent(
                                WorkerEventKind.ERROR,
                                message=(
                                    "idle tick failed: "
                                    f"{type(error).__name__}: {error}"
                                ),
                            )
                        )
                        return
                continue
            if kind == "task":
                self._run_task(session, value)
            elif kind == "home":
                handler = getattr(session, "return_home", None)
                if handler is not None:
                    try:
                        handler(value)
                    except Exception as error:
                        self._log(
                            "return to start failed: "
                            f"{type(error).__name__}: {error}",
                            "WARNING",
                        )
            elif kind == "activate":
                handler = getattr(session, "activate_vehicle", None)
                if handler is not None:
                    try:
                        active = handler(value)
                    except Exception as error:
                        message = (
                            "vehicle activation failed: "
                            f"{type(error).__name__}: {error}"
                        )
                        self._log(message, "WARNING")
                        self._emit(
                            WorkerEvent(
                                WorkerEventKind.ACTIVE,
                                vehicle=value,
                                success=False,
                                message=message,
                            )
                        )
                    else:
                        self._emit(
                            WorkerEvent(
                                WorkerEventKind.ACTIVE,
                                vehicle=active or value,
                                success=True,
                            )
                        )
            elif kind == "view":
                handler = getattr(session, "set_view_mode", None)
                if handler is not None:
                    try:
                        handler(value)
                    except Exception as error:
                        message = (
                            "view change failed: "
                            f"{type(error).__name__}: {error}"
                        )
                        self._log(message, "WARNING")
                        self._emit(
                            WorkerEvent(
                                WorkerEventKind.VIEW,
                                vehicle=value,
                                success=False,
                                message=message,
                            )
                        )
                    else:
                        self._emit(
                            WorkerEvent(
                                WorkerEventKind.VIEW,
                                vehicle=value,
                                success=True,
                            )
                        )
            elif kind == "weather":
                handler = getattr(session, "set_weather", None)
                if handler is not None:
                    try:
                        handler(value)
                    except Exception as error:
                        self._log(
                            "weather change failed: "
                            f"{type(error).__name__}: {error}",
                            "WARNING",
                        )
            elif kind == "shutdown":
                return
            # A "stop" command only marks the event; the running workflow
            # polls it between setpoints.

    def _run_task(self, session, task: TaskSubmission) -> None:
        with self._lock:
            self._stop_event = threading.Event()
            if self._stop_requested:
                self._stop_event.set()
            self._stop_requested = False
            stop_event = self._stop_event
        self._log(f"Task {task.task_id} started: {task.instruction}")
        try:
            result = session.run(
                task, stop_event=stop_event, log=self._log
            )
        except Exception as error:  # keep the worker alive for the next task
            message = f"Task rejected: {type(error).__name__}: {error}"
            self._log(message, "ERROR")
            self._emit(
                WorkerEvent(
                    WorkerEventKind.FINISHED,
                    task_id=task.task_id,
                    message=message,
                    success=False,
                )
            )
            with self._lock:
                self._stop_requested = False
            return
        lines, code = mission_report(result)
        for line in lines:
            self._log(line)
        success = code == 0
        message = lines[-1] if lines else "Task finished."
        if (result.error or "").startswith("cancelled"):
            # The kinematic car always holds the last applied setpoint; the
            # FINISHED event is the stop confirmation the controller waits for.
            self._log(
                f"Stop confirmed for {task.task_id}: "
                "the vehicle holds its last setpoint.",
                "INFO",
            )
        self._emit(
            WorkerEvent(
                WorkerEventKind.FINISHED,
                task_id=task.task_id,
                message=message,
                success=success,
            )
        )
        with self._lock:
            self._stop_requested = False