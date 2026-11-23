"""The trajectory sink a mission child can actually hold.

The console's remaining-path panel reads one hub, and that hub lives in the process
serving HTTP. A family whose simulator client cannot be imported there runs its task
loop in a child process, so the child cannot hold the hub -- but the workflows still
need *a* sink with the hub's interface. Passing ``None`` and letting them publish
nothing is how UE4 lost the panel while UE5 kept it.

Snapshots are built by the real :class:`~planner.runtime.trajectory_hub.TrajectoryHub`
so the fields cannot drift from what the in-process family produces, and publishing is
coalesced: the sliding-window planner can publish several times per control period, and
one pipe line per call would bury the events that matter.
"""

from __future__ import annotations

import time
from collections.abc import Callable

from planner.runtime import TrajectoryHub, TrajectorySnapshot

#: How often a snapshot may cross the pipe while a task is running.
DEFAULT_MIN_INTERVAL_S = 0.4


class RelayedTelemetry:
    """Builds hub-shaped snapshots and forwards them to ``emit``."""

    def __init__(
        self,
        emit: Callable[[TrajectorySnapshot], None],
        *,
        clock=time.monotonic,
        min_interval_s: float = DEFAULT_MIN_INTERVAL_S,
    ) -> None:
        self._emit = emit
        self._clock = clock
        self._min_interval_s = float(min_interval_s)
        self._hub = TrajectoryHub(clock=clock)
        self._sent_at: float | None = None

    # -- the interface the workflows use -----------------------------------
    def start(self, instruction: str) -> TrajectorySnapshot:
        return self._forward(self._hub.start(instruction), force=True)

    def clear(self) -> TrajectorySnapshot:
        # Always sent: leaving a finished path on the console is worse than one
        # extra line, because the operator cannot tell it from a live plan.
        return self._forward(self._hub.clear(), force=True)

    def publish_no_queue(self, **fields) -> TrajectorySnapshot:
        return self._forward(self._hub.publish_no_queue(**fields))

    def publish_queue(self, **fields) -> TrajectorySnapshot:
        return self._forward(self._hub.publish_queue(**fields))

    # -- transport ---------------------------------------------------------
    def _forward(
        self, snapshot: TrajectorySnapshot, *, force: bool = False
    ) -> TrajectorySnapshot:
        now = self._clock()
        if (
            not force
            and self._sent_at is not None
            and now - self._sent_at < self._min_interval_s
        ):
            return snapshot
        self._sent_at = now
        try:
            self._emit(snapshot)
        except Exception:
            # A telemetry channel that cannot be written must not end a flight.
            # The task's own events travel a different path and still report.
            pass
        return snapshot
