from typing import Protocol
from pathlib import Path

from planner.domain.observation import Observation
from planner.domain.visual_goal import GoalObservation, GoalGeometry


class TaskTrace(Protocol):
    path: Path

    def event(self, event: str, **data) -> None: ...
    def observation(self, observation: Observation, goal: GoalObservation) -> None: ...
    def target_geometry(self, observation: Observation, goal: GoalObservation, geometry: GoalGeometry) -> None: ...
