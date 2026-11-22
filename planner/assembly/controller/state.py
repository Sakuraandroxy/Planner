"""Lifecycle state machine, status snapshot, event pump and log sink.

The facade owns the mutable fields. This collaborator is the only place that
reads the state machine for status and the only place that consumes worker
events, so the supervisor thread keeps being the sole writer.
"""

from __future__ import annotations

import logging

from ...domain.server import (
    ServerState,
    ServerStatus,
    WorkerEventKind,
)
from ...domain.view import default_view_for

logger = logging.getLogger(__name__)


class LifecycleState:
    def __init__(self, controller):
        self._c = controller

    @property
    def state(self) -> ServerState:
        with self._c._lock:
            return self._c._state

    @property
    def status(self) -> ServerStatus:
        c = self._c
        with c._lock:
            return ServerStatus(
                c._state,
                c._instruction,
                c._last_result,
                vehicle=c._vehicle,
                vehicles=c._vehicles,
                map=c._map_id,
                maps=c._maps.ids,
                view_mode=c._view_mode,
                view_pending=c._view_pending,
                weather=(
                    dict(c._weather) if c._weather is not None else None
                ),
                map_switching=c._map_switching,
                sim_error=c._sim_error,
                models=(
                    dict(c._models) if c._model_catalog is not None
                    else None
                ),
                models_switching=c._models_switching,
                profile=c._profile,
                profiles=tuple(profile.id for profile in c._profiles),
                profile_switching=c._profile_switching,
            )

    def log(self, text: str, level: str = "INFO") -> None:
        self._c.logs.append(text, level)
        logger.log(getattr(logging, level, logging.INFO), "%s", text)

    def reset_session_state(self) -> None:
        if self._c._session_reset is not None:
            try:
                self._c._session_reset()
            except Exception as error:  # a display reset must never stall a restart
                self.log(f"Session reset failed: {error}", "WARNING")

    def on_event(self) -> None:
        c = self._c
        for _ in range(100):
            worker = c._worker
            if worker is None:
                return
            event = worker.poll()
            if event is None:
                return
            if event.kind is WorkerEventKind.LOG:
                self.log(event.message, event.level)
            elif c.environment.on_event(event):
                pass
            elif event.kind is WorkerEventKind.READY:
                with c._lock:
                    c._worker_ready = True
                    c._state = ServerState.IDLE
                    c._instruction = ""
                    c._pending_activation = None
                    c._view_pending = False
                    previous_view = c._view_mode
                    # A fresh session always starts on the default vehicle:
                    # it hides the other one and calibrates this view itself.
                    # Never replay the previous selection here — it may name
                    # the other vehicle's camera and would leave the console
                    # showing an angle where that vehicle no longer is.
                    c._vehicle = c.settings.default_vehicle
                    c._view_mode = default_view_for(
                        c.settings.default_vehicle
                    )
                    c._applied_view = c._view_mode
                    desired_weather = (
                        dict(c._weather)
                        if c._weather is not None
                        else None
                    )
                    c._weather_applied = False
                self.log("Mission worker ready. State: idle.")
                if previous_view != c._view_mode:
                    self.log(
                        "Initial view reset to "
                        f"{c._view_mode} for the {c._vehicle}."
                    )
                if desired_weather is not None:
                    worker.set_weather(desired_weather)
                    c._weather_applied = True
                    self.log("Reapplied the saved weather state.")
            elif event.kind is WorkerEventKind.FINISHED:
                apply_view = ""
                with c._lock:
                    if event.task_id != c._active or c._state not in (
                        ServerState.RUNNING,
                        ServerState.STOPPING,
                    ):
                        continue
                    c._active = None
                    c._last_result = event.message or (
                        "reached" if event.success else "failed"
                    )
                    c._state = ServerState.IDLE
                    c._instruction = ""
                    if c._view_mode != c._applied_view:
                        apply_view = c._view_mode
                        c._applied_view = apply_view
                        c._view_pending = True
                self.log(
                    f"Task {event.task_id} ended: "
                    f"{event.message or 'finished'}",
                    "INFO" if event.success else "WARNING",
                )
                if apply_view:
                    worker.set_view_mode(apply_view)
                    self.log(f"Applied the deferred streaming view: {apply_view}.")
            elif event.kind is WorkerEventKind.ACTIVE:
                applied = ""
                with c._lock:
                    pending = c._pending_activation
                    c._pending_activation = None
                    c._view_pending = False
                    if event.success:
                        try:
                            applied = default_view_for(
                                event.vehicle or c._vehicle
                            )
                        except ValueError:
                            applied = ""
                        if applied:
                            c._view_mode = applied
                            c._applied_view = applied
                    elif pending is not None:
                        c._vehicle = pending["vehicle"]
                        c._view_mode = pending["view_mode"]
                if event.success:
                    self.log(
                        f"Vehicle activated: {event.vehicle or applied} "
                        f"(view={applied or 'unknown'})."
                    )
                else:
                    self.log(
                        event.message or "Vehicle activation failed.",
                        "WARNING",
                    )
            elif event.kind is WorkerEventKind.ERROR:
                c.commands.failed(
                    event.message or "Worker preparation failed."
                )
                return
            elif event.kind is WorkerEventKind.CLOSED and not c._closing.is_set():
                c.commands.failed("The worker closed the simulator session.")
                return
