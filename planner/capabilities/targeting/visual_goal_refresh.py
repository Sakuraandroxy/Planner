"""Stage-scoped target updates. Only the owner accepts data-only worker results."""
from functools import partial
import threading
import math
import logging

from planner.domain.visual_goal_completion import VisualGoalTargetSnapshot
from planner.services.background_operation import BackgroundOperation
from planner.services.observation_snapshot import freeze_observation, freeze_reference
from .association import make_reference, accept_position, remembered_geometry
from planner.errors import ModelCallTimeout, ProtocolError, BudgetExceeded, ObservationError

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
        geometry = self.geometry.estimate(observation, goal)
        if trace and record_images:
            trace.observation(observation, goal)
            trace.target_geometry(observation, goal, geometry)
        memory, reason = accept_position(session.target_memory, session.stage_id, observation, goal,
            geometry, self.limits.target_position_update_radius_m)
        if reason is not None:
            self.failures += 1
            self.event(trace, 'target_update_rejected', reason=reason, status=goal.status,
                candidate_world_point=geometry.world_point,
                anchor_world_point=memory.anchor_world_point if memory else None,
                retained_world_point=memory.world_point if memory else None,
                anchor_offset_m=(math.dist(memory.anchor_world_point, geometry.world_point)
                    if memory and geometry.world_point is not None else None),
                update_radius_m=self.limits.target_position_update_radius_m,
                using_memory=memory is not None)
            return False
        if session.reference is None:
            session.reference = make_reference(observation, goal, geometry)
        session.target_memory = memory
        session.geometry = remembered_geometry(memory, observation.vehicle_pose)
        self.failures = 0
        self.latest, self.goal = observation, goal
        session.search.found()
        session.trigger.update(VisualGoalTargetSnapshot(session.stage_id, memory.world_point,
                                                       memory.captured_monotonic, stage_memory=True))
        self.event(trace, 'target_updated', capture_id=observation.capture_id, geometry=geometry,
            anchor_world_point=memory.anchor_world_point, position_captured_monotonic=memory.captured_monotonic)
        return True

    def submit(self, observation, parameters, session, trace=None, *, force=False):
        if self.job.busy or self.job.future is not None or (
                not force and self.clock()-self.last_submit < self.limits.target_refresh_interval_s):
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
        except (BudgetExceeded, ObservationError):
            raise
        except (ModelCallTimeout, ProtocolError) as exc:
            if session.target_memory is None:
                raise
            self.failures += 1
            self.event(trace, 'target_update_failed', error_type=type(exc).__name__,
                consecutive_failures=self.failures, using_memory=True)
            return
        if goal is not None:
            self.accept(self.pending_observation, goal, session, trace, record_images=False)

    def observe_stopped(self, observation, parameters, session, trace, checkpoint, sleep):
        """Optional post-binding detector I/O, independent of the Qwen caller."""
        self.poll(session, trace)
        self.submit(observation, parameters, session, trace, force=True)
        while self.job.future is not None:
            checkpoint()
            session.require_time(self.clock)
            self.poll(session, trace)
            if self.job.future is not None:
                sleep(.01)
                # Yield even with a deterministic fake clock; no flight I/O.
                threading.Event().wait(.001)

    def close(self):
        self.job.close()

