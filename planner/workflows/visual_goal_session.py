"""Run-local state and finite control-flow helpers for visual-goal navigation."""
from dataclasses import dataclass
import time

from planner.domain.execution import ExecutionBudget
from planner.domain.visual_goal import GoalReference, GoalGeometry, VisualGoalStatus
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory, MotionTrajectory
from planner.capabilities.completion.visual_goal_trigger import VisualGoalRadiusTrigger
from planner.errors import BudgetExceeded, ModelCallTimeout
from .termination.visual_goal_navigation import VisualGoalExitPolicy, SearchPolicy, VisualGoalFailure
from planner.services.planning_window import PlanningWindow
from planner.domain.planning_window import PlanningWindowInput
from planner.domain.visual_goal import VisualGoalPlanningContext
from planner.services.model_calls import ModelCallService
from typing import Callable


@dataclass
class VisualGoalSession:
    budget: ExecutionBudget
    policy: VisualGoalExitPolicy
    search: SearchPolicy
    started: float
    initial: WorldPose | None = None
    reference: GoalReference | None = None
    geometry: GoalGeometry | None = None
    previous: RelativeTrajectory | None = None
    previous_start: WorldPose | None = None
    window: PlanningWindow[VisualGoalPlanningContext] | None = None
    paused_for_completion: bool = False
    path_length_m: float = 0.0
    translation_pending_progress: bool = False
    confirmations: int = 0
    last_confirmation: float = -float("inf")
    last_timestamp_ns: int = 0
    last_depth_timestamp_ns: int = 0
    last_capture_id: str = ""
    stage_id: str = ""
    trigger: VisualGoalRadiusTrigger | None = None
    pending_motion: MotionTrajectory = MotionTrajectory(())
    completion_checks: int = 0
    completion_confirmed: bool = False
    model_calls: ModelCallService | None = None
    checkpoint: Callable[[], None] | None = None
    target_capture_id: str = ""
    target_captured_monotonic: float | None = None

    def window_input(self) -> PlanningWindowInput:
        if self.initial is None:
            raise ValueError("window request requires a measured stage start pose")
        return PlanningWindowInput(self.initial, self.policy.limits.maximum_observation_age_s,
            self.policy.limits.model_request_timeout_s, self.previous, self.previous_start)

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
            if session.model_calls is None:
                value = operation()
            else:
                timeout = min(limits.model_request_timeout_s, session.budget.remaining(clock()))
                value = session.model_calls.call(operation, timeout, session.checkpoint)
            session.require_time(clock)
            if validate is not None:
                validate(value)
            return value
        except (BudgetExceeded, ModelCallTimeout):
            raise
        except Exception as exc:
            if attempt >= limits.model_retry_count:
                raise VisualGoalFailure(VisualGoalStatus.MODEL_ERROR, str(exc)) from exc
    raise AssertionError("unreachable retry state")
