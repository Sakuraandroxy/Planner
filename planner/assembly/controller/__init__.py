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
from collections.abc import Callable

from ...config import MapCatalog, MapEntry, ProfileInfo, VehicleModelCatalog
from ...domain.server import (
    BackendUnavailable,
    MapSelection,
    ServerState,
    ServerStatus,
    TaskSubmission,
)
from ...domain.view import (
    default_view_for,
    validate_view_mode,
)
from ...domain.weather import WEATHER_CATALOG, merge_weather
from ...errors import SimulatorNotOwned, SimulatorRestartError
from ...ports.backend_worker import BackendWorker
from ...runtime.log_buffer import LogBuffer
from .commands import CommandChannel
from .state import LifecycleState

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
        profiles: tuple[ProfileInfo, ...] = (),
        initial_profile: str = "",
        session_reset: Callable[[], None] | None = None,
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
        # The shared consoles (remaining trajectory, frames) outlive a session, and
        # the simulator processes do not: after a map, appearance-model or
        # configuration switch the vehicles are back at spawn while the console is
        # still drawing the path a previous run was given. Whoever owns those
        # consoles plugs it in here; it runs once the old session is confirmed closed.
        self._session_reset = session_reset
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
        # Navigation profile: the selected one decides which config the next
        # session is assembled from. The simulator process keeps running.
        self._profiles = tuple(profiles)
        self._profile = initial_profile or (
            self._profiles[0].id if self._profiles else ""
        )
        self._profile_switching = False
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
        # Collaborators share the fields above; only the supervisor thread
        # mutates them, HTTP threads read through the properties below.
        self.lifecycle = LifecycleState(self)
        self.commands = CommandChannel(self)

    def start(self) -> None:
        self.commands.start()

    def submit(
        self, instruction: str, vehicle: str | None = None
    ) -> TaskSubmission:
        return self.commands.submit(instruction, vehicle)

    def stop(self) -> ServerState:
        return self.commands.stop()

    def home(self, vehicle: str | None = None) -> str:
        """Teleport a vehicle back to its spawn pose; only while idle."""
        return self.commands.home(vehicle)

    def activate(self, vehicle: str) -> str:
        """Console switch: park the other vehicle, home + show this one.

        The status updates optimistically; a failed ACTIVE event restores the
        previous selection and view.
        """
        return self.commands.activate(vehicle)

    def close(self) -> None:
        self.commands.close()

    @property
    def state(self) -> ServerState:
        return self.lifecycle.state

    @property
    def status(self) -> ServerStatus:
        return self.lifecycle.status

    @property
    def vehicle_models(self) -> VehicleModelCatalog | None:
        return self._model_catalog

    @property
    def profile(self) -> str:
        """The profile the next session will be built from.

        The assembly reads this when it rebuilds, which is how selecting a
        profile reaches both the in-process session and the UE4 child.
        """
        with self._lock:
            return self._profile

    @property
    def profiles(self) -> tuple[ProfileInfo, ...]:
        return self._profiles

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
            self.lifecycle.log(
                "A simulator is running but was not started by the launch "
                "script; leaving it alone. The console may still show its "
                f"screen. {error}",
                "WARNING",
            )
            return
        except SimulatorRestartError as error:
            self.lifecycle.log(
                f"Cannot clean up the running simulator: {error}", "WARNING"
            )
            return
        self.lifecycle.log(
            "Stopping the simulator left over from a previous run."
        )
        try:
            simulator.stop()
        except Exception as error:
            self.lifecycle.log(
                f"Stopping the leftover simulator failed: "
                f"{type(error).__name__}: {error}",
                "ERROR",
            )
            return
        self.lifecycle.log("Leftover simulator stopped; the console starts blank.")

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
        self.lifecycle.log(
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
            self.lifecycle.log(
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
        self.lifecycle.log(f"Preparing map '{entry.id}'; the simulator stack starts.")
        return ServerState.CONNECTING

    def stop_world(self) -> ServerState:
        """Console "stop scene": close the session, then take our stack down.

        Only from ``idle``: a running task owns the session, and the operator can
        stop the task first. The stack is stopped through the same ownership check
        a map switch uses, so a simulator this service did not launch keeps running
        and the console says why.
        """
        if self._simulator is None:
            raise ValueError("this deployment does not manage the simulator")
        with self._lock:
            if self._closing.is_set() or self._state is not ServerState.IDLE:
                raise BackendUnavailable(self._state)
            self._active = None
            self._instruction = ""
            self._last_result = None
            self._pending_activation = None
            self._view_pending = False
            self._weather_applied = False
            self._state = ServerState.CONNECTING
            self._commands.put(("stop_world", self._generation, None))
        self.lifecycle.log("Stopping the scene at the operator's request.")
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
        self.lifecycle.log(
            "Switching vehicle models to "
            + ", ".join(f"{vehicle}={model}" for vehicle, model in cleaned.items())
            + "; reloading the scene."
        )
        return ServerState.CONNECTING

    def request_profile(self, profile: str) -> ServerState:
        """Select a navigation profile.

        With a session running, it is rebuilt from the new file: the profile's values
        are constructor arguments of that session's vehicles and task pipelines, so
        reloading the scene is the only way in. Reloading also puts every vehicle back
        at its spawn, which is what makes two profiles comparable.

        With no session, only the name is recorded; the next scene start reads it.
        """
        if not self._profiles:
            raise ValueError("this deployment does not list any navigation profiles")
        chosen = str(profile).strip()
        if not any(info.id == chosen for info in self._profiles):
            raise ValueError(
                f"unknown profile '{chosen}'; expected one of "
                f"{[info.id for info in self._profiles]}"
            )
        with self._lock:
            if self._closing.is_set():
                raise BackendUnavailable(self._state)
            if chosen == self._profile:
                return self._state
            if self._state is ServerState.DISCONNECTED:
                self._profile = chosen
                self.lifecycle.log(
                    f"Navigation profile '{chosen}' is selected; it is used when a "
                    "scene starts."
                )
                return self._state
            if (
                self._state is not ServerState.IDLE
                or self._worker is None
                or not self._worker_ready
            ):
                raise BackendUnavailable(self._state)
            self._profile = chosen
            self._profile_switching = True
            self._sim_error = None
            self._active = None
            self._instruction = ""
            self._last_result = None
            self._pending_activation = None
            self._view_pending = False
            self._state = ServerState.CONNECTING
            self._commands.put(("profile", self._generation, chosen))
        self.lifecycle.log(
            f"Switching the navigation profile to '{chosen}'; rebuilding the session."
        )
        return ServerState.CONNECTING

    # -- supervisor --------------------------------------------------------

    def _selection_for(self, map_id: str) -> MapSelection:
        entry = self._maps.entry(map_id) or self._maps.maps[0]
        scene_file = entry.scene_file
        if self._model_catalog is not None:
            # Same-map scene variants live on the map entry: a model that has no
            # variant there keeps the map's default scene.
            for group in self._model_catalog.groups:
                model_id = self._models.get(group.vehicle)
                if model_id is not None:
                    scene_file = entry.scene.scene_for(model_id)
        return MapSelection(entry.id, scene_file, entry.sim_config_path)

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
        if not self.commands.close_worker(skip_landing=True):
            with self._lock:
                self._world_retry_at = (
                    self.clock() + self.settings.reconnect_interval_s
                )
            self.lifecycle.log(
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
                self.lifecycle.log(
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
            self.lifecycle.log(
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
        self.lifecycle.log(f"Map switched to '{entry.id}'.")
        self.commands.connect()

    def _stop_world(self) -> None:
        """Worker CLOSED -> stack stop -> disconnected, owner thread."""
        if not self.commands.close_worker(skip_landing=True):
            self.commands.failed(
                "The worker did not confirm shutdown for the scene stop."
            )
            return
        warning = None
        simulator = self._simulator
        if simulator is not None:
            try:
                if simulator.running():
                    simulator.preflight()
                    simulator.stop()
            except SimulatorNotOwned as error:
                warning = (
                    "The scene was not started by this service; it keeps "
                    f"running. {error}"
                )
            except Exception as error:
                warning = (
                    "Stopping the simulator failed: "
                    f"{type(error).__name__}: {error}"
                )
        with self._lock:
            self._state = ServerState.DISCONNECTED
            self._session_started = False
            self._next_start = 0.0
            self._map_switching = False
            self._pending_world = None
            self._worker_ready = False
            self._sim_error = warning
        if warning is not None:
            self.lifecycle.log(warning, "WARNING")
        else:
            self.lifecycle.log("Scene stopped; the console starts blank.")

    def _switch_models(self) -> None:
        """Worker CLOSED -> reconnect with the variant scene, same simulator."""
        if not self.commands.close_worker(skip_landing=True):
            self.commands.failed(
                "The worker did not confirm shutdown for the vehicle-model "
                "switch. Retrying the session."
            )
            return
        with self._lock:
            self._models_switching = False
            self._selection = self._selection_for(self._map_id)
        self.lifecycle.log(
            "Vehicle models switched: "
            + ", ".join(
                f"{vehicle}={model}" for vehicle, model in self._models.items()
            )
            + f"; loading {self._selection.scene_file}."
        )
        self.commands.connect()

    def _switch_profile(self) -> None:
        """Worker CLOSED -> rebuild the session from the profile now selected."""
        if not self.commands.close_worker(skip_landing=True):
            self.commands.failed(
                "The worker did not confirm shutdown for the profile switch. "
                "Retrying the session."
            )
            return
        with self._lock:
            profile = self._profile
            self._profile_switching = False
        self.lifecycle.log(
            f"Navigation profile '{profile}' is active: its planner endpoint and "
            "navigation settings are wired into the new session."
        )
        self.commands.connect()

    def _stop_simulator(self) -> None:
        """Take the simulator stack down when the service goes.

        The stack used to outlive the service, cleaned up only by the next startup. From
        the operator's side that is a leak: Ctrl-C ends the console while UE keeps
        rendering on the xpra port and holding a GPU. The flight console treats this
        differently -- ``cli.py`` turns a signal into ``SystemExit`` and its ``finally``
        shuts the scene manager down -- because the service owns the processes it
        launched. Stopping here goes through the same ladder a map switch uses: xpra
        stop, a verified tree kill if the ports survive, then the display lock.

        A stack this project cannot prove it started is left alone with a warning; the
        ownership rules are the ones that already refuse a map switch.
        """
        simulator = self._simulator
        if simulator is None:
            return
        try:
            if not simulator.running():
                return
        except Exception as error:
            self.lifecycle.log(
                f"Cannot tell whether a simulator is running; leaving it alone "
                f"({type(error).__name__}: {error})",
                "WARNING",
            )
            return
        self.lifecycle.log(
            "Shutting down: stopping the simulator this service started."
        )
        try:
            simulator.stop()
        except SimulatorNotOwned as error:
            self.lifecycle.log(
                "Leaving a simulator this service did not start running. "
                f"{error}",
                "WARNING",
            )
        except Exception as error:
            self.lifecycle.log(
                f"Stopping the simulator on shutdown failed: "
                f"{type(error).__name__}: {error}",
                "ERROR",
            )
