"""Function wrapper for sliding-window Qwen planning."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from agent.core.types import PlanOutput
from agent.models.planner.configured_qwen import IncrementalQwenModel
from agent.models.planner.sliding_window_planner import incremental_to_cumulative


class SlidingWindowPlanningFunction:
    """Call an incremental-body Qwen backend and keep its contract explicit."""

    waypoint_format = "incremental_body"

    def __init__(self, config=None, model=None):
        self.config = config or {}
        self.model = model or IncrementalQwenModel(config)
        self.expected_additional = int(self.config.get("MAX_ADDITIONAL", 5))
        self.min_forward_progress_m = float(
            self.config.get("MIN_FORWARD_PROGRESS_M", 0.0)
        )

    def _validate_output(self, output: PlanOutput) -> str:
        waypoints = list(output.waypoints or [])
        if len(waypoints) <= 0:
            return "no waypoints returned"
        if len(waypoints) > self.expected_additional:
            return f"expected at most {self.expected_additional} waypoints, got {len(waypoints)}"
        if any(
            len(waypoint) < 3
            or any(not math.isfinite(float(value)) for value in waypoint[:3])
            for waypoint in waypoints
        ):
            return "waypoints contain invalid coordinates"
        cumulative = incremental_to_cumulative(waypoints)
        if not cumulative or cumulative[-1][0] <= self.min_forward_progress_m:
            endpoint_x = cumulative[-1][0] if cumulative else 0.0
            return (
                f"endpoint forward progress {endpoint_x:.2f}m is not greater than "
                f"{self.min_forward_progress_m:.2f}m"
            )
        return ""

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
        rejection = self._validate_output(output)
        if rejection:
            output.waypoints = []
            output.reasoning = f"{output.reasoning} | rejected: {rejection}".strip(" |")
            output.rejection_reason = rejection
        return output
