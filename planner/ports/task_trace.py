from typing import Protocol
from pathlib import Path

from planner.domain.observation import Observation
from planner.domain.target_surface import GoalSurfaceObservation, SurfaceSelection


class TaskTrace(Protocol):
    path: Path

    def event(self, event: str, **data) -> None: ...
    def observation(self, observation: Observation, goal: GoalSurfaceObservation) -> None: ...
    def surface_selection(self, observation: Observation, selection: SurfaceSelection) -> None: ...
