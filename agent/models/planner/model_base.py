"""Base interface for waypoint-producing model backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterable, Sequence

from agent.core.types import PlanOutput


class PlannerModel(ABC):
    waypoint_format = "cumulative_body"

    @abstractmethod
    def plan(
        self,
        front_img,
        down_img,
        instruction: str,
        *,
        pending_waypoints: Iterable[Sequence[float]] | None = None,
        **kwargs,
    ) -> PlanOutput:
        ...
