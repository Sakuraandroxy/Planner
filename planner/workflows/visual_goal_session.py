"""Run-local state and finite control-flow helpers for visual-goal navigation."""
from dataclasses import dataclass, field
import time

from planner.domain.execution import ExecutionBudget
from planner.domain.visual_goal import GoalReference, GoalGeometry, VisualGoalStatus
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.target_surface import SurfaceSelection
from planner.errors import BudgetExceeded
from .termination.visual_goal_navigation import VisualGoalExitPolicy, SearchPolicy, VisualGoalFailure


@dataclass
class VisualGoalSession:
    budget: ExecutionBudget
    policy: VisualGoalExitPolicy
    search: SearchPolicy
    started: float
    initial: WorldPose | None = None
    reference: GoalReference | None = None
    geometry: GoalGeometry | None = None
    surface_selection: SurfaceSelection | None = None
    previous: RelativeTrajectory | None = None
    free_balls: list[tuple[tuple[float, float, float], float]] = field(default_factory=list)
    displaced_surface: tuple[float, float, float] | None = None
    path_length_m: float = 0.0
    translation_pending_progress: bool = False
    confirmations: int = 0
    last_confirmation: float = -float("inf")
    last_timestamp_ns: int = 0
    last_depth_timestamp_ns: int = 0
    last_capture_id: str = ""

    def require_time(self, clock=time.monotonic):
        if self.budget.remaining(clock()) <= 0:
            raise BudgetExceeded("task deadline exhausted")

    def fresh(self, observation):
        valid = bool(observation.capture_id) and observation.capture_id != self.last_capture_id
        if observation.timestamp_ns > 0 and observation.timestamp_ns <= self.last_timestamp_ns:
            valid = False
        if observation.depth_timestamp_ns > 0 and observation.depth_timestamp_ns <= self.last_depth_timestamp_ns:
            valid = False
        self.last_timestamp_ns = max(self.last_timestamp_ns, observation.timestamp_ns)
        self.last_depth_timestamp_ns = max(self.last_depth_timestamp_ns, observation.depth_timestamp_ns)
        self.last_capture_id = observation.capture_id
        return valid


def model_call(operation, session, limits, *, is_planning=False, validate=None, clock=time.monotonic):
    for attempt in range(limits.model_retry_count+1):
        session.require_time(clock)
        if is_planning:
            session.policy.planning_call()
        try:
            value = operation()
            session.require_time(clock)
            if validate is not None:
                validate(value)
            return value
        except BudgetExceeded:
            raise
        except Exception as exc:
            if attempt >= limits.model_retry_count:
                raise VisualGoalFailure(VisualGoalStatus.MODEL_ERROR, str(exc)) from exc
    raise AssertionError("unreachable retry state")
