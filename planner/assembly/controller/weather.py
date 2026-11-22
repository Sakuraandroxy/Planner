"""Weather parameters and streaming view: merge, queue, reapply on READY.

Weather and view share one lifecycle: both survive worker reconnects and map
switches, both are replayed once the new session is READY, and while a task
runs the view change is deferred to the task end.
"""

from __future__ import annotations

from ...domain import (
    BackendUnavailable,
    ServerState,
    WorkerEventKind,
    merge_weather,
    validate_view_mode,
)


class EnvironmentChannel:
    def __init__(self, controller):
        self._c = controller

    @property
    def weather_catalog(self):
        c = self._c
        with c._lock:
            return c._weather_catalog

    def request_weather(self, values) -> dict[str, float]:
        """Merge a partial update over the current weather and queue it."""
        c = self._c
        with c._lock:
            if (
                c._closing.is_set()
                or c._state is not ServerState.IDLE
                or c._worker is None
                or not c._worker_ready
            ):
                raise BackendUnavailable(c._state)
            merged = merge_weather(c._weather, values)
            c._weather = merged
            c._commands.put(("weather", c._generation, merged))
        c.lifecycle.log(
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
        c = self._c
        validated = validate_view_mode(mode)
        with c._lock:
            if c._closing.is_set() or c._worker is None:
                raise BackendUnavailable(c._state)
            if c._state is ServerState.IDLE and c._worker_ready:
                c._view_mode = validated
                c._view_pending = True
                c._commands.put(("view", c._generation, validated))
                return validated, False
            if c._state in (ServerState.RUNNING, ServerState.STOPPING):
                c._view_mode = validated
                return validated, True
            raise BackendUnavailable(c._state)

    def apply_view(self, mode: str, worker) -> None:
        """Owner thread: hand a requested view to the worker."""
        c = self._c
        c.lifecycle.log(f"Applying streaming view: {mode}.")
        worker.set_view_mode(mode)
        c._applied_view = mode

    def apply_weather(self, values, worker) -> None:
        """Owner thread: hand the merged weather to the worker."""
        c = self._c
        worker.set_weather(values)
        c._weather_applied = True

    def accept_catalog(self, event) -> None:
        c = self._c
        with c._lock:
            if event.payload is not None:
                c._weather_catalog = event.payload

    def on_event(self, event) -> bool:
        """Handle the WEATHER and VIEW events; True when this event was one."""
        c = self._c
        if event.kind is WorkerEventKind.WEATHER:
            self.accept_catalog(event)
            return True
        if event.kind is WorkerEventKind.VIEW:
            with c._lock:
                c._view_pending = False
                if event.success:
                    c._applied_view = event.vehicle or c._view_mode
                elif c._state is ServerState.IDLE:
                    c._view_mode = c._applied_view
            if not event.success:
                c.lifecycle.log(
                    event.message or "View change failed.", "WARNING"
                )
            return True
        return False
