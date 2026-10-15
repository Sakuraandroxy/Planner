"""Stage-scoped target updates. Only the owner accepts data-only worker results."""
from dataclasses import replace
from functools import partial
import math
import logging

from planner.domain.visual_goal import GoalVisibility
from planner.domain.visual_goal_completion import VisualGoalTargetSnapshot
from planner.services.background_operation import BackgroundOperation
from planner.services.observation_snapshot import freeze_observation, freeze_reference
from .association import make_reference
from planner.errors import ModelCallTimeout

logger = logging.getLogger(__name__)


def _observe_and_trace(observer, geometry, trace, observation, description, reference, **kwargs):
    """Data-only worker: diagnostic image encoding must not stall control."""
    goal = observer.observe(observation, description, reference, **kwargs)
    if trace is not None:
        try:
            trace.observation(observation, goal)
            trace.target_geometry(observation, goal, geometry.estimate(observation, goal))
        except Exception:
            logger.exception("Target diagnostic image write failed")
    return goal


class VisualGoalRefresh:
    def __init__(self, observer, geometry, limits, clock, event):
        self.observer, self.geometry, self.limits = observer, geometry, limits
        self.clock, self.event = clock, event
        self.job = BackgroundOperation(clock)
        self.pending_observation = None
        self.last_submit = -float('inf')
        self.latest = self.goal = None
        self.failures = 0

    def begin(self):
        if not self.job.busy:
            self.job.discard()
        self.job.require_available()
        self.last_submit = -float('inf')
        self.latest = self.goal = self.pending_observation = None
        self.failures = 0

    def accept(self, observation, goal, session, trace, *, record_images=True):
        if self.latest is not None and observation.timestamp_ns <= self.latest.timestamp_ns:
            return False
        geometry = self.geometry.estimate(observation, goal)
        if trace and record_images:
            trace.observation(observation, goal)
            trace.target_geometry(observation, goal, geometry)
        if goal.status is not GoalVisibility.VISIBLE or not geometry.reliable or not goal.same_target:
            self.failures += 1
            self.event(trace, 'target_update_rejected', reason=geometry.reason, status=goal.status)
            return False
        if session.reference is not None and math.dist(session.reference.world_point, geometry.world_point) > self.limits.maximum_target_position_jump_m:
            self.failures += 1
            self.event(trace, 'target_identity_uncertain', old_position=session.reference.world_point,
                candidate_position=geometry.world_point, maximum_jump_m=self.limits.maximum_target_position_jump_m)
            return False
        session.reference = (make_reference(observation, goal, geometry) if session.reference is None else
            replace(session.reference, world_point=geometry.world_point))
        session.geometry = geometry
        self.failures = 0
        self.latest, self.goal = observation, goal
        session.trigger.update(VisualGoalTargetSnapshot(session.stage_id, geometry.world_point, observation.captured_monotonic))
        self.event(trace, 'target_updated', capture_id=observation.capture_id, geometry=geometry)
        return True

    def submit(self, observation, parameters, session, trace=None):
        if self.job.busy or self.job.future is not None or self.clock()-self.last_submit < self.limits.target_refresh_interval_s:
            return
        if self.latest is not None and observation.timestamp_ns <= self.latest.timestamp_ns:
            return
        self.pending_observation = observation
        operation = partial(_observe_and_trace, self.observer, self.geometry, trace,
            freeze_observation(observation), parameters.target_description,
            freeze_reference(session.reference), initial_pose=session.initial,
            budget=session.budget, instruction=parameters.instruction)
        self.job.start(operation, min(self.limits.model_request_timeout_s, session.budget.remaining(self.clock())))
        self.last_submit = self.clock()

    def poll(self, session, trace):
        try:
            goal = self.job.poll()
        except ModelCallTimeout:
            raise
        except Exception as exc:
            self.failures += 1
            self.event(trace, 'target_update_failed', error_type=type(exc).__name__, consecutive_failures=self.failures)
            return
        if goal is not None:
            self.accept(self.pending_observation, goal, session, trace, record_images=False)

    def close(self):
        self.job.close()

