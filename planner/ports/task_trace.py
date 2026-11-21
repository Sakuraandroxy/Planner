from pathlib import Path
from typing import Protocol

from planner.domain.observation import Observation
from planner.domain.visual_goal import GoalGeometry, GoalObservation


class TaskTrace(Protocol):
    path: Path

    def event(self, event: str, **data) -> None: ...
    def observation(self, observation: Observation, goal: GoalObservation) -> None: ...
    def target_geometry(self, observation: Observation, goal: GoalObservation, geometry: GoalGeometry) -> None: ...