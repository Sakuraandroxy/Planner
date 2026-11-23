"""Thread-safe publisher for the console's remaining-trajectory panel."""

from __future__ import annotations

import threading
import time
from dataclasses import replace

from ..domain.pose import WorldPose
from ..domain.trajectory_telemetry import (
    MODE_IDLE,
    MODE_NO_QUEUE,
    MODE_PENDING,
    QueueSnapshot,
    TrajectorySnapshot,
    build_queue_snapshot,
    vehicle_snapshot,
)


class TrajectoryHub:
    """Publish immutable snapshots from the workflow thread; read from HTTP."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._snapshot = TrajectorySnapshot()
        #: Accumulated vehicle poses for the current task, tagged with the stage
        #: that produced them. The live snapshot above is only the *remaining*
        #: path; this list is the condensed flight history the console's eval
        #: save button reads. Reset when a new task is submitted.
        self._history: list[dict] = []

    def read(self) -> TrajectorySnapshot:
        with self._lock:
            return self._snapshot

    def reset_history(self) -> None:
        with self._lock:
            self._history = []

    def history(self) -> list[dict]:
        with self._lock:
            return [dict(point) for point in self._history]

    def publish(self, snapshot: TrajectorySnapshot) -> TrajectorySnapshot:
        with self._lock:
            published = replace(
                snapshot, revision=self._snapshot.revision + 1
            )
            self._snapshot = published
            vehicle = published.vehicle
            if vehicle is not None:
                self._history.append({
                    "stage_id": published.stage_id,
                    "x": vehicle.x, "y": vehicle.y, "z": vehicle.z,
                })
                # A single task cannot legitimately fly hundreds of thousands of
                # samples; cap so a runaway publisher cannot grow memory forever.
                if len(self._history) > 50000:
                    del self._history[: len(self._history) - 50000]
            return published

    def adopt(self, snapshot: TrajectorySnapshot) -> TrajectorySnapshot:
        """Publish a snapshot produced somewhere else -- a mission child.

        The child cannot hold this hub, so it sends the snapshot over the worker
        channel and the parent adopts it here. The child's ``updated_monotonic``
        is meaningless across processes, so the parent stamps its own clock.
        """
        return self.publish(
            replace(snapshot, updated_monotonic=self._clock())
        )

    def start(self, instruction: str) -> TrajectorySnapshot:
        """Mark a task as active before its first stage publishes a queue."""
        return self.publish(
            TrajectorySnapshot(
                active=True,
                instruction=instruction,
                mode=MODE_PENDING,
                updated_monotonic=self._clock(),
            )
        )

    def clear(self) -> TrajectorySnapshot:
        return self.publish(
            TrajectorySnapshot(
                active=False,
                mode=MODE_IDLE,
                updated_monotonic=self._clock(),
            )
        )

    def publish_no_queue(
        self,
        *,
        instruction: str,
        stage_id: str | None,
        pose: WorldPose | None = None,
        mode: str = MODE_NO_QUEUE,
    ) -> TrajectorySnapshot:
        return self.publish(
            TrajectorySnapshot(
                active=True,
                instruction=instruction,
                stage_id=stage_id,
                mode=mode,
                queue=QueueSnapshot(0, 0),
                vehicle=None if pose is None else vehicle_snapshot(pose),
                updated_monotonic=self._clock(),
            )
        )

    def publish_queue(
        self,
        *,
        instruction: str,
        stage_id: str | None,
        mode: str,
        capacity: int,
        pose: WorldPose,
        items,
        planning_calls: int,
        path_length_m: float,
        vertical_motion: bool = True,
    ) -> TrajectorySnapshot:
        return self.publish(
            build_queue_snapshot(
                instruction=instruction,
                stage_id=stage_id,
                mode=mode,
                capacity=capacity,
                pose=pose,
                items=items,
                planning_calls=planning_calls,
                path_length_m=path_length_m,
                clock=self._clock,
                vertical_motion=vertical_motion,
            )
        )