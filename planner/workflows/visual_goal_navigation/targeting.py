"""Visual-goal targeting: background refresh of the target and bbox observation.

只做检测与状态更新，不决定完成与否；完成判定在 completion.py。
"""
import logging
import math
import threading
import time
from functools import partial

from planner.algorithms.targeting import (
    accept_position,
    bound_target_geometry,
    make_reference,
)
from planner.domain import ExecutionBudget
from planner.domain import Observation
from planner.domain import TargetDetectionInput
from planner.domain import TargetObservationEvidence
from planner.domain import GoalObservation, GoalReference, GoalVisibility
from planner.domain import VisualGoalTargetSnapshot
from planner.errors import (
    BudgetExceeded,
    ModelCallTimeout,
    ObservationError,
    ProtocolError,
)
from planner.ports import TargetDetector
from planner.runtime import BackgroundOperation
from planner.runtime import freeze_observation, freeze_reference
from planner.workflows.common.budget import require_time

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
        target_state, reason = accept_position(session.target_state, session.stage_id, observation, goal,
            geometry, self.limits.target_position_update_radius_m)
        previous = session.target_observation
        if previous is None or observation.timestamp_ns > previous.timestamp_ns:
            session.target_observation = TargetObservationEvidence(session.stage_id, observation.capture_id,
                observation.captured_monotonic, observation.timestamp_ns, goal, reason)
        if reason is not None:
            self.failures += 1
            self.event(trace, 'target_update_rejected', reason=reason, status=goal.status,
                candidate_world_point=geometry.world_point,
                anchor_world_point=target_state.anchor_world_point if target_state else None,
                retained_world_point=target_state.world_point if target_state else None,
                anchor_offset_m=(math.dist(target_state.anchor_world_point, geometry.world_point)
                    if target_state and geometry.world_point is not None else None),
                update_radius_m=self.limits.target_position_update_radius_m,
                using_bound_target=target_state is not None)
            return False
        if session.reference is None or session.reference.crop is None:
            session.reference = make_reference(observation, goal, geometry)
        session.target_state = target_state
        session.geometry = bound_target_geometry(target_state, observation.pose)
        self.failures = 0
        self.latest, self.goal = observation, goal
        session.search.found()
        session.trigger.update(VisualGoalTargetSnapshot(session.stage_id, target_state.world_point,
            target_state.captured_monotonic, stage_bound=True, surface_points_world=target_state.known_surfaces))
        self.event(trace, 'target_updated', capture_id=observation.capture_id, geometry=geometry,
            retained_surface_points_world=target_state.known_surfaces,
            initial_surface_points_world=target_state.anchor_surface_points_world,
            anchor_world_point=target_state.anchor_world_point, position_captured_monotonic=target_state.captured_monotonic)
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
            if session.target_state is None:
                raise
            self.failures += 1
            self.event(trace, 'target_update_failed', error_type=type(exc).__name__,
                consecutive_failures=self.failures, using_bound_target=True)
            return
        if goal is not None:
            self.accept(self.pending_observation, goal, session, trace, record_images=False)

    def observe_stopped(self, observation, parameters, session, trace, checkpoint, sleep):
        """Optional post-binding detector I/O, independent of the Qwen caller."""
        self.poll(session, trace)
        self.submit(observation, parameters, session, trace, force=True)
        while self.job.future is not None:
            checkpoint()
            require_time(session.budget, self.clock)
            self.poll(session, trace)
            if self.job.future is not None:
                sleep(.01)
                # Yield even with a deterministic fake clock; no flight I/O.
                threading.Event().wait(.001)

    def close(self):
        self.job.close()


# Optional grounding observer; surface pixels are selected by the model.
class BBoxGoalObserver:
    def __init__(self, detector: TargetDetector, timeout_s: float = 20.0, clock=time.monotonic):
        self.detector, self.timeout_s, self.clock = detector, timeout_s, clock

    def observe(self, observation: Observation, target_description: str,
                reference: GoalReference | None = None, *, budget: ExecutionBudget | None = None,
                initial_pose=None, instruction: str | None = None) -> GoalObservation:
        timeout = self.timeout_s if budget is None else min(self.timeout_s, budget.remaining(self.clock()))
        if timeout <= 0:
            raise BudgetExceeded("deadline before target detection")
        request = TargetDetectionInput(observation, instruction or target_description,
            target_description, reference.crop if reference is not None else None)
        detection = self.detector.detect(request, timeout_s=timeout)
        if budget is not None and budget.remaining(self.clock()) <= 0:
            raise BudgetExceeded("late target detection discarded")
        source = "model_reference_surface_points" if reference is not None else "model_surface_points"
        if not detection.found:
            return GoalObservation(GoalVisibility.NOT_VISIBLE, None, None, target_description,
                "detector returned no certain bbox", False, source, surface_points=())
        box = detection.bbox.coordinates
        return GoalObservation(GoalVisibility.VISIBLE, box, None, target_description,
            "model target association and visible surface samples; not persistent physical landmarks",
            True, source, surface_points=detection.surface_points)
