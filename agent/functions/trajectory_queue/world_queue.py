"""World-coordinate trajectory queue for sliding-window control."""

from __future__ import annotations

from typing import Iterable, List, Sequence

from agent.models.planner.sliding_window_planner import (
    SlidingWindowTrajectoryQueue,
    cumulative_body_to_world,
    cumulative_to_incremental,
    incremental_to_cumulative,
    world_to_cumulative_body,
)


class WorldTrajectoryQueue(SlidingWindowTrajectoryQueue):
    """Queue wrapper for absolute world waypoints and incremental-body Qwen I/O."""

    def pending_world(self) -> List[List[float]]:
        pending_count = max(0, int(self.max_pending))
        return [list(wp) for wp in self.world_waypoints[:pending_count]]

    def pending_for_model(
        self,
        current_pos: Sequence[float],
        current_yaw_deg: float,
        current_rot_body_to_world=None,
    ) -> List[List[float]]:
        return self.pending_incremental(
            current_pos,
            current_yaw_deg,
            current_rot_body_to_world=current_rot_body_to_world,
        )

    def append_model_output(
        self,
        additional_incremental: Iterable[Sequence[float]] | None,
        anchor_pos: Sequence[float],
        anchor_yaw_deg: float,
        anchor_world_waypoints: Iterable[Sequence[float]] | None = None,
        anchor_rot_body_to_world=None,
    ) -> int:
        return self.append_incremental_output(
            additional_incremental,
            anchor_pos,
            anchor_yaw_deg,
            anchor_world_waypoints=anchor_world_waypoints,
            anchor_rot_body_to_world=anchor_rot_body_to_world,
        )

    def execution_slice(
        self,
        current_pos: Sequence[float],
        current_yaw_deg: float,
        count: int | None = None,
        current_rot_body_to_world=None,
    ) -> List[List[float]]:
        n = int(count if count is not None else self.execute_count)
        n = max(1, n)
        return [list(wp) for wp in self.world_waypoints[:n]]


__all__ = [
    "WorldTrajectoryQueue",
    "cumulative_body_to_world",
    "cumulative_to_incremental",
    "incremental_to_cumulative",
    "world_to_cumulative_body",
]
