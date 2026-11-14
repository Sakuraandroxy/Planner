"""Thread-safe publisher for the console's remaining-trajectory panel."""

from __future__ import annotations

from dataclasses import replace
import threading
import time

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

    def read(self) -> TrajectorySnapshot:
        with self._lock:
            return self._snapshot

    def publish(self, snapshot: TrajectorySnapshot) -> TrajectorySnapshot:
        with self._lock:
            published = replace(
                snapshot, revision=self._snapshot.revision + 1
            )
            self._snapshot = published
            return published

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
            )
        )