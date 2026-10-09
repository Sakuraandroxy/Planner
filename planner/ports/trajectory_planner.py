from typing import Protocol

from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import VisualGoalPlanningContext
from planner.domain.execution import ExecutionBudget


class TrajectoryPlanner(Protocol):
    """Model continuation interface; pending is reserved for future queue integration.

    Current runtime supplies None or an empty trajectory. A nonempty pending queue
    needs snapshot ownership and execution-anchor support before it may be used.

    Model implementations return exactly five NEW relative pose actions, excluding
    pending. Unsupported pending or invalid output raises ProtocolError; HTTP
    failures propagate as backend request errors and exhausted budgets fail closed.
    Local basic-motion/search trajectories are outside this model-output contract.
    """
    def plan(
        self,
        observation: Observation,
        instruction: str,
        *,
        current_guidance: str | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
        pending_trajectory: RelativeTrajectory | None = None,
        visual_goal_context: VisualGoalPlanningContext | None = None,
        budget: ExecutionBudget | None = None,
    ) -> RelativeTrajectory: ...

