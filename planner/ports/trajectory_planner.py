from typing import Protocol

from planner.domain.observation import Observation
from planner.domain.trajectory import RelativeTrajectory


class TrajectoryPlanner(Protocol):
    def plan(self, observation: Observation, instruction: str) -> RelativeTrajectory: ...

