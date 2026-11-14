from typing import Protocol, TypeVar

from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.execution import ExecutionBudget


ContextT = TypeVar("ContextT", contravariant=True)


class TrajectoryPlanner(Protocol[ContextT]):
    """Plan five NEW actions after a frozen 0..5-point unexecuted pending prefix.

    Serial runtime supplies an empty trajectory. Sliding runtime supplies the
    owner-frozen remainder; its result is anchored to that snapshot's tail and
    cannot be executed before current Workflow authorization.

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
        guidance_start: WorldPose | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
        previous_trajectory_start: WorldPose | None = None,
        pending_trajectory: RelativeTrajectory | None = None,
        task_context: ContextT | None = None,
        budget: ExecutionBudget | None = None,
        timeout_s: float | None = None,
    ) -> RelativeTrajectory: ...

