from typing import Protocol

from planner.domain import ExecutionBudget
from planner.domain import Observation
from planner.domain import WorldPose
from planner.domain import GoalObservation, GoalReference


class GoalObserver(Protocol):
    def observe(self, observation: Observation, target_description: str,
                reference: GoalReference | None = None, *,
                budget: ExecutionBudget | None = None,
                initial_pose: WorldPose | None = None, instruction: str | None = None) -> GoalObservation: ...