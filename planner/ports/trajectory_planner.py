from typing import Protocol

from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory


class TrajectoryPlanner(Protocol):
    def plan(
        self,
        observation: Observation,
        instruction: str,
        *,
        current_guidance: str | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
    ) -> RelativeTrajectory: ...

