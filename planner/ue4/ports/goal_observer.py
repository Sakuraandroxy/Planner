from typing import Protocol

from planner.ue4.domain.execution import ExecutionBudget
from planner.ue4.domain.observation import Observation
from planner.ue4.domain.pose import WorldPose
from planner.ue4.domain.visual_goal import GoalReference
from planner.ue4.domain.visual_goal import GoalObservation


class GoalObserver(Protocol):
    def observe(self, observation: Observation, target_description: str,
                reference: GoalReference | None = None, *,
                budget: ExecutionBudget | None = None,
                initial_pose: WorldPose | None = None, instruction: str | None = None) -> GoalObservation: ...
