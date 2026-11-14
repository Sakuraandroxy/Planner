"""One backend lifecycle owner; HTTP callers only submit commands and read data.

The supervisor thread owns the worker. A worker failure or simulator connect
error transitions back to ``connecting`` and retries after
``reconnect_interval_s``.

A map switch is different: ProjectAirSim cannot replace a cooked UE map in a
running process, so the controller stops the worker (the topic socket is a
single-consumer link), stops the whole simulator stack through the ownership-
checked supervisor, launches the target map and reconnects. While the switch
runs the service stays ``connecting`` and every other command is rejected.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable

from ..config import MapCatalog, MapEntry, VehicleModelCatalog
from ..domain.server import (
    BackendUnavailable,
    MapSelection,
    ServerState,
    ServerStatus,
    TaskSubmission,
    WorkerEventKind,
)
from ..domain.view import (
    default_view_for,
    validate_view_mode,
)
from ..domain.weather import WEATHER_CATALOG, merge_weather
from ..errors import SimulatorNotOwned, SimulatorRestartError
from ..ports.backend_worker import BackendWorker
from ..services.log_buffer import LogBuffer

logger = logging.getLogger(__name__)


class ServerController:
    def __init__(
        self,
        worker_factory: Callable[[MapSelection], BackendWorker],
        settings,
        *,
        maps: MapCatalog,
        simulator=None,
        initial_map: str | None = None,
        vehicle_models: VehicleModelCatalog | None = None,
        restart_attempts: int = 2,
        worker_shutdown_timeout_s: float = 60.0,
        autostart_session: bool = True,
        clock=time.monotonic,
    ):
        self.worker_factory = worker_factory
        self.settings = settings
        self.clock = clock
        self.logs = LogBuffer(settings.log_buffer_size)
        self._maps = maps
        self._simulator = simulator
        # Production starts disconnected: no worker is created and no socket
        # is opened until the console asks for a map. autostart_session keeps
        # the immediate-connect behavior for tests and headless runs.
        self._session_started = bool(autostart_session)
        self._restart_attempts = max(1, int(restart_attempts))
        self._worker_shutdown_timeout_s = float(worker_shutdown_timeout_s)
        self._state = (
            ServerState.CONNECTING
            if autostart_session
            else ServerState.DISCONNECTED
        )
        self._lock = threading.Lock()
        self._commands: queue.Queue = queue.Queue()
        self._closing = threading.Event()
        self._thread: threading.Thread | None = None
        self._worker: BackendWorker | None = None
        self._worker_ready = False
        self._generation = 0
        self._sequence = 0
        self._active: str | None = None
        self._instruction = ""
        self._last_result: str | None = None
        self._next_start = 0.0
        self._started_at = 0.0
        # Task-level vehicle routing: every submission names one accepted id.
        self._vehicles = tuple(settings.vehicles)
        self._vehicle = settings.default_vehicle
        # Appearance models per vehicle. Changing one reloads the scene in the
        # same simulator process; a map switch keeps the current selection.
        self._model_catalog = vehicle_models
        self._models = (
            dict(vehicle_models.defaults) if vehicle_models is not None else {}
        )
        self._models_switching = False
        # Map lifecycle.
        map_id = initial_map if self._maps.entry(initial_map or "") else (
            self._maps.default_map
        )
        self._map_id = map_id
        self._selection = self._selection_for(map_id)
        self._map_switching = False
        self._sim_error: str | None = None
        self._pending_world: MapEntry | None = None
        # True when the requested map needs stop+launch (a stack is already
        # running); False means the stack is absent and is simply launched.
        self._world_restart = False
        self._world_retry_at = 0.0
        # Weather/view state survives worker reconnects and map switches; the
        # worker applies it once the new session is READY.
        self._weather_catalog = WEATHER_CATALOG
        self._weather: dict[str, float] | None = None
        self._weather_applied = False
        # The session calibrates the streaming-view cycle against the live
        # scene and parks on the default vehicle's chase (tail) camera, so both
        # fields start there instead of assuming a fixed cycle index.
        self._view_mode = default_view_for(settings.default_vehicle)
        self._applied_view = self._view_mode
        # True while the worker is cycling the streaming camera to a requested
        # view; the console shows a blank screen during the cycle.
        self._view_pending = False
        # Previous selection while a console vehicle activation is in flight;
        # used to roll back the optimistic status when it fails.
        self._pending_activation: dict[str, str] | None = None

    @property
    def state(self) -> ServerState:
        with self._lock:
            return self._state

    @property
    def status(self) -> ServerStatus:
        with self._lock:
            return ServerStatus(
                self._state,
                self._instruction,
                self._last_result,
                vehicle=self._vehicle,
                vehicles=self._vehicles,
                map=self._map_id,
                maps=self._maps.ids,
                view_mode=self._view_mode,
                view_pending=self._view_pending,
                weather=(
                    dict(self._weather) if self._weather is not None else None
                ),
                map_switching=self._map_switching,
                sim_error=self._sim_error,
                models=(
                    dict(self._models) if self._model_catalog is not None
                    else None
                ),
                models_switching=self._models_switching,
            )

    @property
    def vehicle_models(self) -> VehicleModelCatalog | None:
        return self._model_catalog

    @property
    def maps(self) -> MapCatalog:
        return self._maps

    @property
    def simulator_owned(self) -> bool:
        """True when the running stack passes the launch-script ownership check."""
        if self._simulator is None:
            return False
        try:
            self._simulator.preflight()
        except (SimulatorNotOwned, SimulatorRestartError):
            return False
        return True

    @property
    def weather_catalog(self):
        with self._lock:
            return self._weather_catalog

    def start(self) -> None:
        if self._thread is not None:
            return
        if not self._session_started:
            self._stop_leftover_simulator()
            self._log(
                "No scene is loaded. Choose a map in the console to start "
                "the simulator."
            )
        self._thread = threading.Thread(
            target=self._run, name="planner-supervisor", daemon=True
        )
        self._thread.start()

    def _stop_leftover_simulator(self) -> None:
        """Stop a script-managed stack so a disconnected start shows no scene.

        Only processes whose pidfiles and command lines match this project are
        touched. A simulator started outside the launch script is left running
        (the console can attach to it after a map is chosen).
        """
        simulator = self._simulator
        if simulator is None or not simulator.running():
            return
        try:
            simulator.preflight()
        except SimulatorNotOwned as error:
            self._log(
                "A simulator is running but was not started by the launch "
                "script; leaving it alone. The console may still show its "
                f"screen. {error}",
                "WARNING",
            )
            return
        except SimulatorRestartError as error:
            self._log(
                f"Cannot clean up the running simulator: {error}", "WARNING"
            )
            return
        self._log("Stopping the simulator left over from a previous run.")
        try:
            simulator.stop()
        except Exception as error:
            self._log(
                f"Stopping the leftover simulator failed: "
                f"{type(error).__name__}: {error}",
                "ERROR",
            )
            return
        self._log("Leftover simulator stopped; the console starts blank.")

    def submit(self, instruction: str, vehicle: str | None = None) -> TaskSubmission:
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("instruction must be non-empty text")
        requested = self._vehicle if vehicle is None else str(vehicle).strip()
        if requested not in self._vehicles:
            raise ValueError(
                f"unknown vehicle '{requested}'; expected one of "
                f"{list(self._vehicles)}"
            )
        with self._lock:
            if self._closing.is_set() or self._state is not ServerState.IDLE:
                raise BackendUnavailable(self._state)
            self._sequence += 1
            task = TaskSubmission(
                f"task_{self._sequence}",
                instruction.strip(),
                requested,
            )
            self._active = task.task_id
            self._instruction = task.instruction
            self._vehicle = requested
            self._state = ServerState.RUNNING
            self._commands.put(("task", self._generation, task))
        return task

    def stop(self) -> ServerState:
        with self._lock:
            if self._state is ServerState.RUNNING:
                self._state = ServerState.STOPPING
                self._commands.put(("stop", self._generation, None))
            return self._state

    def home(self, vehicle: str | None = None) -> str:
        """Teleport a vehicle back to its spawn pose; only while idle."""
        with self._lock:
            if (
                self._closing.is_set()
                or self._state is not ServerState.IDLE
                or self._worker is None
                or not self._worker_ready
            ):
                raise BackendUnavailable(self._state)
            requested = self._vehicle if vehicle is None else str(vehicle).strip()
            if requested not in self._vehicles:
                raise ValueError(
                    f"unknown vehicle '{requested}'; expected one of "
                    f"{list(self._vehicles)}"
                )
            self._commands.put(("home", self._generation, requested))
            return requested

    def activate(self, vehicle: str) -> str:
        """Console switch: park the other vehicle, home + show this one.

        The status updates optimistically; a failed ACTIVE event restores the
        previous selection and view.
        """
        with self._lock:
            if (
                self._closing.is_set()
                or self._state is not ServerState.IDLE
                or self._worker is None
                or not self._worker_ready
            ):
                raise BackendUnavailable(self._state)
            requested = str(vehicle).strip()
            if requested not in self._vehicles:
                raise ValueError(
                    f"unknown vehicle '{requested}'; expected one of "
                    f"{list(self._vehicles)}"
                )
            self._pending_activation = {
                "vehicle": self._vehicle,
                "view_mode": self._view_mode,
            }
            self._vehicle = requested
            self._view_mode = default_view_for(requested)
            self._view_pending = True
            self._commands.put(("activate", self._generation, requested))
        return requested

    def request_weather(self, values) -> dict[str, float]:
        """Merge a partial update over the current weather and queue it."""
        with self._lock:
            if (
                self._closing.is_set()
                or self._state is not ServerState.IDLE
                or self._worker is None
                or not self._worker_ready
            ):
                raise BackendUnavailable(self._state)
            merged = merge_weather(self._weather, values)
            self._weather = merged
            self._commands.put(("weather", self._generation, merged))
        self._log(
            "Weather request: "
            + ", ".join(
                f"{name}={value:.2f}"
                for name, value in merged.items()
                if value > 0.0
            )
        )
        return merged

    def request_view(self, mode: str) -> tuple[str, bool]:
        """Select a streaming view; running tasks defer it to the task end."""
        validated = validate_view_mode(mode)
        with self._lock:
            if self._closing.is_set() or self._worker is None:
                raise BackendUnavailable(self._state)
            if self._state is ServerState.IDLE and self._worker_ready:
                self._view_mode = validated
                self._view_pending = True
                self._commands.put(("view", self._generation, validated))
                return validated, False
            if self._state in (ServerState.RUNNING, ServerState.STOPPING):
                self._view_mode = validated
                return validated, True
            raise BackendUnavailable(self._state)

    def request_world(self, map_id: str) -> ServerState:
        """Start a scene or switch the running simulator to another map.

        From ``disconnected`` this is the console's "start scene": an absent
        stack is launched, and a stack already running the selected map is
        simply attached to. From ``idle`` it restarts the stack for the new
        map. Preflight runs before any state change so a simulator that was
        not started by the launch script is rejected with an actionable error
        instead of being killed.
        """
        entry = self._maps.entry(str(map_id))
        if entry is None:
            raise ValueError(
                f"unknown map '{map_id}'; expected one of {list(self._maps.ids)}"
            )
        if self._simulator is None:
            raise ValueError("this deployment does not manage the simulator")
        attach = False
        with self._lock:
            if self._closing.is_set() or self._state not in (
                ServerState.DISCONNECTED,
                ServerState.IDLE,
            ):
                raise BackendUnavailable(self._state)
            # From idle a session is connected, so the stack is by definition
            # running and must go through stop+launch. From disconnected the
            # ports say whether some stack is already up (for another map).
            restart = (
                self._state is ServerState.IDLE or self._simulator.running()
            )
            if self._state is ServerState.IDLE:
                if self._worker is None or not self._worker_ready:
                    raise BackendUnavailable(self._state)
                if entry.id == self._map_id:
                    return self._state
            elif entry.id == self._map_id and self._simulator.running():
                # The planner restarted while the same map kept running:
                # connect instead of paying for another simulator restart.
                self._state = ServerState.CONNECTING
                self._session_started = True
                self._sim_error = None
                attach = True
        if attach:
            self._log(
                f"Attaching to the running simulator for '{entry.id}'."
            )
            return ServerState.CONNECTING
        if restart:
            try:
                self._simulator.preflight()
            except Exception as error:
                raise ValueError(f"cannot switch maps: {error}") from error
        with self._lock:
            if self._state not in (
                ServerState.DISCONNECTED,
                ServerState.IDLE,
            ):
                raise BackendUnavailable(self._state)
            self._map_switching = True
            self._sim_error = None
            self._active = None
            self._instruction = ""
            self._last_result = None
            self._pending_activation = None
            self._view_pending = False
            self._world_restart = restart
            self._state = ServerState.CONNECTING
            self._session_started = True
            self._commands.put(("world", self._generation, entry))
        self._log(f"Preparing map '{entry.id}'; the simulator stack starts.")
        return ServerState.CONNECTING

    def request_models(self, updates: dict[str, str]) -> ServerState:
        """Select appearance models and reload the scene (idle only).

        The simulator process and the loaded cooked map stay untouched; only
        the worker is restarted so the new session loads the matching scene
        variant. Every vehicle resets to its spawn pose.
        """
        catalog = self._model_catalog
        if catalog is None:
            raise ValueError("this deployment does not configure vehicle models")
        if not isinstance(updates, dict) or not updates:
            raise ValueError("models must name at least one vehicle")
        cleaned: dict[str, str] = {}
        for vehicle, model_id in updates.items():
            vehicle_id = str(vehicle).strip()
            group = catalog.group(vehicle_id)
            if group is None:
                raise ValueError(
                    f"unknown vehicle '{vehicle_id}'; expected one of "
                    f"{list(catalog.vehicles)}"
                )
            chosen = str(model_id).strip()
            if group.option(chosen) is None:
                raise ValueError(
                    f"unknown {vehicle_id} model '{chosen}'; expected one of "
                    f"{list(group.ids)}"
                )
            cleaned[vehicle_id] = chosen
        with self._lock:
            if (
                self._closing.is_set()
                or self._state is not ServerState.IDLE
                or self._worker is None
                or not self._worker_ready
            ):
                raise BackendUnavailable(self._state)
            merged = dict(self._models)
            merged.update(cleaned)
            if merged == self._models:
                return self._state
            self._models = merged
            self._models_switching = True
            self._sim_error = None
            self._active = None
            self._instruction = ""
            self._last_result = None
            self._pending_activation = None
            self._view_pending = False
            self._state = ServerState.CONNECTING
            self._commands.put(("models", self._generation, None))
        self._log(
            "Switching vehicle models to "
            + ", ".join(f"{vehicle}={model}" for vehicle, model in cleaned.items())
            + "; reloading the scene."
        )
        return ServerState.CONNECTING

    # -- supervisor --------------------------------------------------------
    def _log(self, text: str, level: str = "INFO") -> None:
        self.logs.append(text, level)
        logger.log(getattr(logging, level, logging.INFO), "%s", text)

    def _selection_for(self, map_id: str) -> MapSelection:
        entry = self._maps.entry(map_id) or self._maps.maps[0]
        scene_file = entry.scene_file
        if self._model_catalog is not None:
            # Same-map scene variants come from the selected model options;
            # options without a variant keep the map's default scene.
            for group in self._model_catalog.groups:
                model_id = self._models.get(group.vehicle)
                option = group.option(model_id) if model_id is not None else None
                if option is not None:
                    scene_file = option.scene_for(entry.id, scene_file)
        return MapSelection(entry.id, scene_file, entry.sim_config_path)

    def _connect(self) -> None:
        try:
            worker = self.worker_factory(self._selection)
        except Exception as error:
            self._log(
                f"Worker construction failed: {type(error).__name__}: {error}",
                "ERROR",
            )
            self._next_start = self.clock() + self.settings.reconnect_interval_s
            return
        with self._lock:
            self._generation += 1
            self._worker = worker
            self._worker_ready = False
            self._started_at = self.clock()
            self._state = ServerState.CONNECTING
        worker.start()

    def _failed_worker(self, reason: str) -> None:
        self._log(reason, "ERROR")
        worker, self._worker = self._worker, None
        if worker is not None:
            try:
                worker.terminate()
            except Exception:
                pass
        with self._lock:
            self._generation += 1
            self._active = None
            self._worker_ready = False
            self._state = ServerState.CONNECTING
            self._instruction = ""
            self._weather_applied = False
            self._pending_activation = None
            self._view_pending = False
        self._next_start = self.clock() + self.settings.reconnect_interval_s

    def _command(self, command) -> None:
        kind, generation, value = command
        with self._lock:
            if generation != self._generation:
                return
            state, active = self._state, self._active
            worker = self._worker
        if kind == "world":
            self._switch_world(value)
            return
        if kind == "models":
            self._switch_models()
            return
        if worker is None:
            return
        if kind == "task" and state is ServerState.RUNNING and value.task_id == active:
            self._log(f"Task {value.task_id} accepted: {value.instruction}")
            worker.submit(value)
        elif kind == "stop" and state is ServerState.STOPPING:
            self._log("Stopping the task; motion aborts between setpoints.")
            worker.stop()
        elif kind == "home" and state is ServerState.IDLE:
            self._log(f"Returning the {value} to its start position.")
            worker.home(value)
        elif kind == "activate" and state is ServerState.IDLE:
            self._log(
                f"Activating the {value}: parking the other vehicle and "
                "returning it home."
            )
            worker.activate(value)
        elif kind == "view" and state is ServerState.IDLE:
            self._log(f"Applying streaming view: {value}.")
            worker.set_view_mode(value)
            self._applied_view = value
        elif kind == "weather" and state is ServerState.IDLE:
            worker.set_weather(value)
            self._weather_applied = True

    def _apply_simulator(self, entry: MapEntry, restart: bool) -> None:
        """Launch an absent stack, or restart the running one for a new map."""
        if restart:
            self._simulator.restart(entry)
        else:
            self._simulator.start(entry)

    def _switch_world(self, entry: MapEntry) -> None:
        """Worker CLOSED -> stack start/restart -> reconnect, owner thread."""
        with self._lock:
            self._pending_world = entry
            self._map_switching = True
        if not self._close_worker(skip_landing=True):
            with self._lock:
                self._world_retry_at = (
                    self.clock() + self.settings.reconnect_interval_s
                )
            self._log(
                "The worker did not confirm shutdown; the map switch will "
                "retry without touching the simulator.",
                "WARNING",
            )
            return
        last_error: Exception | None = None
        for attempt in range(1, self._restart_attempts + 1):
            try:
                self._apply_simulator(entry, self._world_restart)
            except Exception as error:
                last_error = error
                self._log(
                    f"Simulator attempt {attempt}/{self._restart_attempts} "
                    f"failed: {type(error).__name__}: {error}",
                    "ERROR",
                )
                continue
            last_error = None
            break
        if last_error is not None:
            with self._lock:
                self._sim_error = (
                    f"simulator start for '{entry.id}' failed: {last_error}"
                )
                self._map_switching = False
                self._pending_world = None
                # Back to the unconnected state so the console can retry the
                # same scene after the operator fixes the stack.
                self._state = ServerState.DISCONNECTED
                self._session_started = False
                self._next_start = 0.0
            self._log(
                "Map start failed; fix the simulator and choose the scene "
                "again to retry (see /api/status sim_error).",
                "ERROR",
            )
            return
        with self._lock:
            self._map_id = entry.id
            self._selection = self._selection_for(entry.id)
            self._map_switching = False
            self._sim_error = None
            self._pending_world = None
        self._log(f"Map switched to '{entry.id}'.")
        self._connect()

    def _switch_models(self) -> None:
        """Worker CLOSED -> reconnect with the variant scene, same simulator."""
        if not self._close_worker(skip_landing=True):
            self._failed_worker(
                "The worker did not confirm shutdown for the vehicle-model "
                "switch. Retrying the session."
            )
            return
        with self._lock:
            self._models_switching = False
            self._selection = self._selection_for(self._map_id)
        self._log(
            "Vehicle models switched: "
            + ", ".join(
                f"{vehicle}={model}" for vehicle, model in self._models.items()
            )
            + f"; loading {self._selection.scene_file}."
        )
        self._connect()

    def _close_worker(self, *, skip_landing: bool = False) -> bool:
        """Shut the worker down and wait for CLOSED before a restart."""
        with self._lock:
            worker = self._worker
        if worker is None:
            with self._lock:
                self._worker_ready = False
            return True
        try:
            worker.shutdown(skip_landing=skip_landing)
        except Exception as error:
            self._log(f"Worker shutdown failed: {error}", "ERROR")
            try:
                worker.terminate()
            except Exception:
                pass
        if not self._wait_closed(worker):
            return False
        with self._lock:
            if self._worker is worker:
                self._worker = None
            self._worker_ready = False
        return True

    def _wait_closed(self, worker) -> bool:
        deadline = self.clock() + self._worker_shutdown_timeout_s
        closed = False
        while True:
            if self._drain(worker):
                closed = True
            if closed:
                return True
            if self.clock() >= deadline:
                return False
            time.sleep(0.05)

    def _drain(self, worker) -> bool:
        """Consume available events; return True when CLOSED was seen."""
        closed = False
        for _ in range(100):
            event = worker.poll()
            if event is None:
                break
            if event.kind is WorkerEventKind.LOG:
                self._log(event.message, event.level)
            elif event.kind is WorkerEventKind.CLOSED:
                closed = True
            elif event.kind is WorkerEventKind.WEATHER:
                with self._lock:
                    if event.payload is not None:
                        self._weather_catalog = event.payload
        return closed

    def _events(self) -> None:
        for _ in range(100):
            worker = self._worker
            if worker is None:
                return
            event = worker.poll()
            if event is None:
                return
            if event.kind is WorkerEventKind.LOG:
                self._log(event.message, event.level)
            elif event.kind is WorkerEventKind.WEATHER:
                with self._lock:
                    if event.payload is not None:
                        self._weather_catalog = event.payload
            elif event.kind is WorkerEventKind.READY:
                with self._lock:
                    self._worker_ready = True
                    self._state = ServerState.IDLE
                    self._instruction = ""
                    self._pending_activation = None
                    self._view_pending = False
                    previous_view = self._view_mode
                    # A fresh session always starts on the default vehicle:
                    # it hides the other one and calibrates this view itself.
                    # Never replay the previous selection here — it may name
                    # the other vehicle's camera and would leave the console
                    # showing an angle where that vehicle no longer is.
                    self._vehicle = self.settings.default_vehicle
                    self._view_mode = default_view_for(
                        self.settings.default_vehicle
                    )
                    self._applied_view = self._view_mode
                    desired_weather = (
                        dict(self._weather)
                        if self._weather is not None
                        else None
                    )
                    self._weather_applied = False
                self._log("Mission worker ready. State: idle.")
                if previous_view != self._view_mode:
                    self._log(
                        "Initial view reset to "
                        f"{self._view_mode} for the {self._vehicle}."
                    )
                if desired_weather is not None:
                    worker.set_weather(desired_weather)
                    self._weather_applied = True
                    self._log("Reapplied the saved weather state.")
            elif event.kind is WorkerEventKind.FINISHED:
                apply_view = ""
                with self._lock:
                    if event.task_id != self._active or self._state not in (
                        ServerState.RUNNING,
                        ServerState.STOPPING,
                    ):
                        continue
                    self._active = None
                    self._last_result = event.message or (
                        "reached" if event.success else "failed"
                    )
                    self._state = ServerState.IDLE
                    self._instruction = ""
                    if self._view_mode != self._applied_view:
                        apply_view = self._view_mode
                        self._applied_view = apply_view
                        self._view_pending = True
                self._log(
                    f"Task {event.task_id} ended: "
                    f"{event.message or 'finished'}",
                    "INFO" if event.success else "WARNING",
                )
                if apply_view:
                    worker.set_view_mode(apply_view)
                    self._log(f"Applied the deferred streaming view: {apply_view}.")
            elif event.kind is WorkerEventKind.ACTIVE:
                applied = ""
                with self._lock:
                    pending = self._pending_activation
                    self._pending_activation = None
                    self._view_pending = False
                    if event.success:
                        try:
                            applied = default_view_for(
                                event.vehicle or self._vehicle
                            )
                        except ValueError:
                            applied = ""
                        if applied:
                            self._view_mode = applied
                            self._applied_view = applied
                    elif pending is not None:
                        self._vehicle = pending["vehicle"]
                        self._view_mode = pending["view_mode"]
                if event.success:
                    self._log(
                        f"Vehicle activated: {event.vehicle or applied} "
                        f"(view={applied or 'unknown'})."
                    )
                else:
                    self._log(
                        event.message or "Vehicle activation failed.",
                        "WARNING",
                    )
            elif event.kind is WorkerEventKind.VIEW:
                with self._lock:
                    self._view_pending = False
                    if event.success:
                        self._applied_view = event.vehicle or self._view_mode
                    elif self._state is ServerState.IDLE:
                        self._view_mode = self._applied_view
                if not event.success:
                    self._log(
                        event.message or "View change failed.", "WARNING"
                    )
            elif event.kind is WorkerEventKind.ERROR:
                self._failed_worker(
                    event.message or "Worker preparation failed."
                )
                return
            elif event.kind is WorkerEventKind.CLOSED:
                if not self._closing.is_set():
                    self._failed_worker("The worker closed the simulator session.")
                    return

    def _run(self) -> None:
        try:
            while not self._closing.is_set():
                try:
                    command = self._commands.get(timeout=0.05)
                except queue.Empty:
                    command = None
                try:
                    if command is not None:
                        self._command(command)
                    if (
                        self._pending_world is not None
                        and self.clock() >= self._world_retry_at
                    ):
                        self._switch_world(self._pending_world)
                    if (
                        self._session_started
                        and self._worker is None
                        and self.clock() >= self._next_start
                    ):
                        self._connect()
                    self._events()
                    if (
                        self._worker is not None
                        and not self._worker_ready
                        and self.clock() - self._started_at
                        >= self.settings.startup_timeout_s
                    ):
                        self._failed_worker(
                            "Simulator session setup timed out. Retrying."
                        )
                except Exception as error:
                    self._failed_worker(
                        f"Backend lifecycle error: {type(error).__name__}: "
                        f"{error}"
                    )
        finally:
            self._shutdown_worker()

    def _shutdown_worker(self) -> None:
        worker, self._worker = self._worker, None
        with self._lock:
            self._worker_ready = False
        if worker is None:
            return
        try:
            worker.shutdown()
        except Exception as error:
            self._log(f"Worker shutdown failed: {error}", "ERROR")
            try:
                worker.terminate()
            except Exception:
                pass
        if not self._drain(worker):
            self._log(
                "Worker did not confirm shutdown within the timeout.", "ERROR"
            )

    def close(self) -> None:
        with self._lock:
            if self._closing.is_set():
                return
            self._pending_activation = None
            self._view_pending = False
            self._state = ServerState.STOPPING
            self._closing.set()
        if self._thread is not None:
            self._thread.join(timeout=self.settings.startup_timeout_s + 8.0)
            if self._thread.is_alive():
                raise RuntimeError("planner supervisor did not finish shutdown")
