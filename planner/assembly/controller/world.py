"""Map selection, simulator stack start/stop, session rebuild after a switch.

A map switch is different from a worker failure: ProjectAirSim cannot replace
a cooked UE map in a running process, so the controller stops the worker (the
topic socket is a single-consumer link), stops the whole simulator stack
through the ownership-checked supervisor, launches the target map and
reconnects. While the switch runs the service stays ``connecting`` and every
other command is rejected.
"""

from __future__ import annotations

from ...config import MapCatalog, MapEntry
from ...domain.server import (
    BackendUnavailable,
    MapSelection,
    ServerState,
)
from ...errors import SimulatorNotOwned, SimulatorRestartError


class SessionSwitcher:
    def __init__(self, controller):
        self._c = controller

    @property
    def maps(self) -> MapCatalog:
        return self._c._maps

    @property
    def simulator_owned(self) -> bool:
        """True when the running stack passes the launch-script ownership check."""
        simulator = self._c._simulator
        if simulator is None:
            return False
        try:
            simulator.preflight()
        except (SimulatorNotOwned, SimulatorRestartError):
            return False
        return True

    def stop_leftover_simulator(self) -> None:
        """Stop a script-managed stack so a disconnected start shows no scene.

        Only processes whose pidfiles and command lines match this project are
        touched. A simulator started outside the launch script is left running
        (the console can attach to it after a map is chosen).
        """
        c = self._c
        simulator = c._simulator
        if simulator is None or not simulator.running():
            return
        try:
            simulator.preflight()
        except SimulatorNotOwned as error:
            c.lifecycle.log(
                "A simulator is running but was not started by the launch "
                "script; leaving it alone. The console may still show its "
                f"screen. {error}",
                "WARNING",
            )
            return
        except SimulatorRestartError as error:
            c.lifecycle.log(
                f"Cannot clean up the running simulator: {error}", "WARNING"
            )
            return
        c.lifecycle.log(
            "Stopping the simulator left over from a previous run."
        )
        try:
            simulator.stop()
        except Exception as error:
            c.lifecycle.log(
                f"Stopping the leftover simulator failed: "
                f"{type(error).__name__}: {error}",
                "ERROR",
            )
            return
        c.lifecycle.log("Leftover simulator stopped; the console starts blank.")

    def request_world(self, map_id: str) -> ServerState:
        """Start a scene or switch the running simulator to another map.

        From ``disconnected`` this is the console's "start scene": an absent
        stack is launched, and a stack already running the selected map is
        simply attached to. From ``idle`` it restarts the stack for the new
        map. Preflight runs before any state change so a simulator that was
        not started by the launch script is rejected with an actionable error
        instead of being killed.
        """
        c = self._c
        entry = c._maps.entry(str(map_id))
        if entry is None:
            raise ValueError(
                f"unknown map '{map_id}'; expected one of {list(c._maps.ids)}"
            )
        if c._simulator is None:
            raise ValueError("this deployment does not manage the simulator")
        attach = False
        with c._lock:
            if c._closing.is_set() or c._state not in (
                ServerState.DISCONNECTED,
                ServerState.IDLE,
            ):
                raise BackendUnavailable(c._state)
            # From idle a session is connected, so the stack is by definition
            # running and must go through stop+launch. From disconnected the
            # ports say whether some stack is already up (for another map).
            restart = c._state is ServerState.IDLE or c._simulator.running()
            if c._state is ServerState.IDLE:
                if c._worker is None or not c._worker_ready:
                    raise BackendUnavailable(c._state)
                if entry.id == c._map_id:
                    return c._state
            elif entry.id == c._map_id and c._simulator.running():
                # The planner restarted while the same map kept running:
                # connect instead of paying for another simulator restart.
                c._state = ServerState.CONNECTING
                c._session_started = True
                c._sim_error = None
                attach = True
        if attach:
            c.lifecycle.log(
                f"Attaching to the running simulator for '{entry.id}'."
            )
            return ServerState.CONNECTING
        if restart:
            try:
                c._simulator.preflight()
            except Exception as error:
                raise ValueError(f"cannot switch maps: {error}") from error
        with c._lock:
            if c._state not in (
                ServerState.DISCONNECTED,
                ServerState.IDLE,
            ):
                raise BackendUnavailable(c._state)
            c._map_switching = True
            c._sim_error = None
            c._active = None
            c._instruction = ""
            c._last_result = None
            c._pending_activation = None
            c._view_pending = False
            c._world_restart = restart
            c._state = ServerState.CONNECTING
            c._session_started = True
            c._commands.put(("world", c._generation, entry))
        c.lifecycle.log(
            f"Preparing map '{entry.id}'; the simulator stack starts."
        )
        return ServerState.CONNECTING

    def stop_world(self) -> ServerState:
        """Console "stop scene": close the session, then take our stack down.

        Only from ``idle``: a running task owns the session, and the operator can
        stop the task first. The stack is stopped through the same ownership check
        a map switch uses, so a simulator this service did not launch keeps running
        and the console says why.
        """
        c = self._c
        if c._simulator is None:
            raise ValueError("this deployment does not manage the simulator")
        with c._lock:
            if c._closing.is_set() or c._state is not ServerState.IDLE:
                raise BackendUnavailable(c._state)
            c._active = None
            c._instruction = ""
            c._last_result = None
            c._pending_activation = None
            c._view_pending = False
            c._weather_applied = False
            c._state = ServerState.CONNECTING
            c._commands.put(("stop_world", c._generation, None))
        c.lifecycle.log("Stopping the scene at the operator's request.")
        return ServerState.CONNECTING

    def selection_for(self, map_id: str) -> MapSelection:
        c = self._c
        entry = c._maps.entry(map_id) or c._maps.maps[0]
        scene_file = entry.scene_file
        if c._model_catalog is not None:
            # Same-map scene variants live on the map entry: a model that has no
            # variant there keeps the map's default scene.
            for group in c._model_catalog.groups:
                model_id = c._models.get(group.vehicle)
                if model_id is not None:
                    scene_file = entry.scene.scene_for(model_id)
        return MapSelection(entry.id, scene_file, entry.sim_config_path)

    def apply_simulator(self, entry: MapEntry, restart: bool) -> None:
        """Launch an absent stack, or restart the running one for a new map."""
        if restart:
            self._c._simulator.restart(entry)
        else:
            self._c._simulator.start(entry)

    def switch_world(self, entry: MapEntry) -> None:
        """Worker CLOSED -> stack start/restart -> reconnect, owner thread."""
        c = self._c
        with c._lock:
            c._pending_world = entry
            c._map_switching = True
        if not c.commands.close_worker(skip_landing=True):
            with c._lock:
                c._world_retry_at = (
                    c.clock() + c.settings.reconnect_interval_s
                )
            c.lifecycle.log(
                "The worker did not confirm shutdown; the map switch will "
                "retry without touching the simulator.",
                "WARNING",
            )
            return
        last_error: Exception | None = None
        for attempt in range(1, c._restart_attempts + 1):
            try:
                self.apply_simulator(entry, c._world_restart)
            except Exception as error:
                last_error = error
                c.lifecycle.log(
                    f"Simulator attempt {attempt}/{c._restart_attempts} "
                    f"failed: {type(error).__name__}: {error}",
                    "ERROR",
                )
                continue
            last_error = None
            break
        if last_error is not None:
            with c._lock:
                c._sim_error = (
                    f"simulator start for '{entry.id}' failed: {last_error}"
                )
                c._map_switching = False
                c._pending_world = None
                # Back to the unconnected state so the console can retry the
                # same scene after the operator fixes the stack.
                c._state = ServerState.DISCONNECTED
                c._session_started = False
                c._next_start = 0.0
            c.lifecycle.log(
                "Map start failed; fix the simulator and choose the scene "
                "again to retry (see /api/status sim_error).",
                "ERROR",
            )
            return
        with c._lock:
            c._map_id = entry.id
            c._selection = self.selection_for(entry.id)
            c._map_switching = False
            c._sim_error = None
            c._pending_world = None
        c.lifecycle.log(f"Map switched to '{entry.id}'.")
        c.commands.connect()

    def stop_world_command(self) -> None:
        """Worker CLOSED -> stack stop -> disconnected, owner thread."""
        c = self._c
        if not c.commands.close_worker(skip_landing=True):
            c.commands.failed(
                "The worker did not confirm shutdown for the scene stop."
            )
            return
        warning = None
        simulator = c._simulator
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
        with c._lock:
            c._state = ServerState.DISCONNECTED
            c._session_started = False
            c._next_start = 0.0
            c._map_switching = False
            c._pending_world = None
            c._worker_ready = False
            c._sim_error = warning
        if warning is not None:
            c.lifecycle.log(warning, "WARNING")
        else:
            c.lifecycle.log("Scene stopped; the console starts blank.")

    def stop_simulator(self) -> None:
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
        c = self._c
        simulator = c._simulator
        if simulator is None:
            return
        try:
            if not simulator.running():
                return
        except Exception as error:
            c.lifecycle.log(
                f"Cannot tell whether a simulator is running; leaving it alone "
                f"({type(error).__name__}: {error})",
                "WARNING",
            )
            return
        c.lifecycle.log(
            "Shutting down: stopping the simulator this service started."
        )
        try:
            simulator.stop()
        except SimulatorNotOwned as error:
            c.lifecycle.log(
                "Leaving a simulator this service did not start running. "
                f"{error}",
                "WARNING",
            )
        except Exception as error:
            c.lifecycle.log(
                f"Stopping the simulator on shutdown failed: "
                f"{type(error).__name__}: {error}",
                "ERROR",
            )
