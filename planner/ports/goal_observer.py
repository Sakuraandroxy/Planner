from typing import Protocol

from planner.domain.execution import ExecutionBudget
from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.visual_goal import GoalObservation, GoalReference


class GoalObserver(Protocol):
    def observe(self, observation: Observation, target_description: str,
                reference: GoalReference | None = None, *,
                budget: ExecutionBudget | None = None,
                initial_pose: WorldPose | None = None) -> GoalObservation: ...
