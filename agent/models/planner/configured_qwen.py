"""Model wrappers for the configured Qwen planners."""

from __future__ import annotations

from typing import Iterable, Sequence

from agent.core.types import PlanOutput
from agent.models.planner import build_planner
from agent.models.planner.model_base import PlannerModel
from agent.models.planner.sliding_window_planner import SlidingWindowQwenPlanner


class CumulativeQwenModel(PlannerModel):
    """Qwen planner that outputs cumulative body-frame waypoints."""

    waypoint_format = "cumulative_body"

    def __init__(self, config=None, planner=None):
        self.config = config or {}
        self.planner = planner or build_planner()

    def plan(self, front_img, down_img, instruction: str, *, pending_waypoints=None, **kwargs) -> PlanOutput:
        result = self.planner.plan(front_img, down_img, instruction=instruction, **kwargs)
        return PlanOutput(
            waypoints=[list(wp) for wp in (result.waypoints or [])],
            waypoint_format=self.waypoint_format,
            done=bool(result.done),
            reasoning=result.reasoning,
            raw=result,
        )


class IncrementalQwenModel(PlannerModel):
    """Sliding-window Qwen planner that outputs incremental body-frame deltas."""

    waypoint_format = "incremental_body"

    def __init__(self, config=None, planner=None):
        self.config = config or {}
        self.planner = planner or SlidingWindowQwenPlanner()

    def plan(
        self,
        front_img,
        down_img,
        instruction: str,
        *,
        pending_waypoints: Iterable[Sequence[float]] | None = None,
        **kwargs,
    ) -> PlanOutput:
        result = self.planner.plan(
            front_img,
            down_img,
            instruction=instruction,
            pending_waypoints=pending_waypoints,
            **kwargs,
        )
        return PlanOutput(
            waypoints=[list(wp) for wp in (result.waypoints or [])],
            waypoint_format=self.waypoint_format,
            done=bool(result.done),
            reasoning=result.reasoning,
            raw=result,
        )
