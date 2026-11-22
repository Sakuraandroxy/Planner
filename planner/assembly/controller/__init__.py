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

import queue
import threading
import time
from collections.abc import Callable

from ...config import MapCatalog, MapEntry, ProfileInfo, VehicleModelCatalog
from ...domain import (
    WEATHER_CATALOG,
    Capabilities,
    MapSelection,
    ServerState,
    ServerStatus,
    TaskSubmission,
    default_view_for,
)
from ...runtime import LogBuffer
from ...simulator import BackendWorker, registry
from .commands import CommandChannel
from .models import ModelSwitcher
from .profiles import ProfileSwitcher
from .state import LifecycleState
from .weather import EnvironmentChannel
from .world import SessionSwitcher


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
        # Collaborators share the fields below; only the supervisor thread
        # mutates them, HTTP threads read through the properties.
        self.lifecycle = LifecycleState(self)
        self.commands = CommandChannel(self)
        self.world = SessionSwitcher(self)
        self.models = ModelSwitcher(self)
        self.profile_switcher = ProfileSwitcher(self)
        self.environment = EnvironmentChannel(self)
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
        self._selection = self.world.selection_for(map_id)
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

    def request_world(self, map_id: str) -> ServerState:
        """Start a scene or switch the running simulator to another map.

        From ``disconnected`` this is the console's "start scene": an absent
        stack is launched, and a stack already running the selected map is
        simply attached to. From ``idle`` it restarts the stack for the new
        map. Preflight runs before any state change so a simulator that was
        not started by the launch script is rejected with an actionable error
        instead of being killed.
        """
        return self.world.request_world(map_id)

    def stop_world(self) -> ServerState:
        """Console "stop scene": close the session, then take our stack down.

        Only from ``idle``: a running task owns the session, and the operator can
        stop the task first. The stack is stopped through the same ownership check
        a map switch uses, so a simulator this service did not launch keeps running
        and the console says why.
        """
        return self.world.stop_world()

    @property
    def maps(self) -> MapCatalog:
        return self.world.maps

    @property
    def simulator_owned(self) -> bool:
        """True when the running stack passes the launch-script ownership check."""
        return self.world.simulator_owned

    def capabilities(self) -> Capabilities | None:
        """Console flags for the active map's simulator.

        The requirement table these flags derive from is the task packages'
        declarations, read by the assembly; the interface layer asks this method
        and names no simulator itself. ``None`` means the active map has no
        catalog entry, so nothing can be gated.
        """
        entry = self._maps.entry(self.status.map)
        if entry is None:
            return None
        return registry.capabilities_for(entry.simulator)

    @property
    def active_simulator(self) -> str:
        """Simulator driving the active map; empty when the catalog lacks it."""
        entry = self._maps.entry(self.status.map)
        return entry.simulator if entry is not None else ""

    @property
    def simulator_labels(self) -> dict[str, str]:
        return registry.labels()

    @property
    def state(self) -> ServerState:
        return self.lifecycle.state

    @property
    def status(self) -> ServerStatus:
        return self.lifecycle.status

    @property
    def vehicle_models(self) -> VehicleModelCatalog | None:
        return self.models.vehicle_models

    def request_models(self, updates: dict[str, str]) -> ServerState:
        """Select appearance models and reload the scene (idle only).

        The simulator process and the loaded cooked map stay untouched; only
        the worker is restarted so the new session loads the matching scene
        variant. Every vehicle resets to its spawn pose.
        """
        return self.models.request_models(updates)

    @property
    def weather_catalog(self):
        return self.environment.weather_catalog

    def request_weather(self, values) -> dict[str, float]:
        """Merge a partial update over the current weather and queue it."""
        return self.environment.request_weather(values)

    def request_view(self, mode: str) -> tuple[str, bool]:
        """Select a streaming view; running tasks defer it to the task end."""
        return self.environment.request_view(mode)

    @property
    def profile(self) -> str:
        """The profile the next session will be built from.

        The assembly reads this when it rebuilds, which is how selecting a
        profile reaches both the in-process session and the UE4 child.
        """
        return self.profile_switcher.profile

    @property
    def profiles(self) -> tuple[ProfileInfo, ...]:
        return self.profile_switcher.profiles

    def request_profile(self, profile: str) -> ServerState:
        """Select a navigation profile.

        With a session running, it is rebuilt from the new file: the profile's values
        are constructor arguments of that session's vehicles and task pipelines, so
        reloading the scene is the only way in. Reloading also puts every vehicle back
        at its spawn, which is what makes two profiles comparable.

        With no session, only the name is recorded; the next scene start reads it.
        """
        return self.profile_switcher.request_profile(profile)

