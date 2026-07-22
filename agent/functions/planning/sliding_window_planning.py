"""Function wrapper for sliding-window Qwen planning."""

from __future__ import annotations

from typing import Iterable, Sequence

from agent.core.types import PlanOutput
from agent.models.planner.configured_qwen import IncrementalQwenModel


class SlidingWindowPlanningFunction:
    """Call an incremental-body Qwen backend and keep its contract explicit."""

    waypoint_format = "incremental_body"

    def __init__(self, config=None, model=None):
        self.config = config or {}
        self.model = model or IncrementalQwenModel(config)

    def plan(
        self,
        front_img,
        down_img,
        instruction: str,
        *,
        pending_waypoints: Iterable[Sequence[float]] | None = None,
        **kwargs,
    ) -> PlanOutput:
        output = self.model.plan(
            front_img,
            down_img,
            instruction,
            pending_waypoints=pending_waypoints,
            **kwargs,
        )
        output.waypoint_format = self.waypoint_format
        return output
