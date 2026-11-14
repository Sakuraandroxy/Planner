"""Above orchestration over unchanged capture, streaming and planning services."""
import time
import logging
from typing import Callable

from planner.domain.planning_window import PlanningWindowInput
from planner.domain.streaming import QueueObservationAnchor
from planner.errors import BudgetExceeded, ExecutionError, ProtocolError
from planner.services.continuous_execution import ContinuousExecution
from planner.services.moving_observations import MovingObservations
from planner.services.planning_window import PlanningWindow
from planner.services.model_calls import ModelCallService
from planner.services.background_operation import BackgroundOperation
from planner.services.resource_cleanup import close_resources
from planner.ports.observation_source import ObservationSource

logger = logging.getLogger(__name__)


class AboveFlight:
    def __init__(self, execution, holding, planner, validator, source, sliding, motion_limits,
                 speed, tracking, settings, *, stopped_source: ObservationSource,
                 close_stopped_source: Callable[[], None], clock=time.monotonic, sleep=time.sleep):
        self.execution, self.holding, self.settings = execution,holding,settings
        self.clock, self.sleep = clock,sleep
        self.window = PlanningWindow(planner,validator,sliding,motion_limits,clock)
        self.stream = ContinuousExecution(execution,execution.vehicle,motion_limits,speed,clock,
                                         settings.model_request_timeout_s,settings=tracking)
        self.frames = MovingObservations(source,clock,settings.model_request_timeout_s)
        self.detection = BackgroundOperation(clock)
        self.calls = ModelCallService(clock)
        self.stopped_source, self.close_stopped_source = stopped_source,close_stopped_source
        self.latest = None
        self.observation = None
        self.last_capture_request = -float("inf")

    def pending(self):
        return (self.window.worker.has_pending_call or self.frames.job.busy or
                self.detection.busy or self.calls.has_pending_call)

    def begin(self, session):
        self.window.begin(deadline_monotonic=session.budget.deadline_monotonic,
                          maximum_planning_rounds=self.settings.maximum_planning_calls)
        self.frames.begin()
        self.stream.begin(self.window.queue,self.window.background.stage_token)
        self.latest = None
        self.observation = None
        self.last_capture_request = -float("inf")

    def checkpoint(self):
        self.execution.checkpoint()
        self.execution.check_deadline()

    def stop(self):
        result = self.execution.stop()
        if result.errors:
            raise ExecutionError("above flight could not hold: "+"; ".join(result.errors))

    def drain(self):
        # Detach results before waiting. Running HTTP is not force-cancelled or
        # replaced by another worker; the hold remains active while it returns.
        self.window.background.discard()
        self.frames.discard()
        self.detection.discard()
        while self.pending():
            self.checkpoint()
            self.sleep(.01)

    def pause(self, session, trace, reason):
        self.stream.pause(reason)
        started = self.clock()
        self.drain()
        drained = self.clock()
        self.holding.wait(budget=session.budget,checkpoint=self.checkpoint)
        self.stream.sample()
        trace.event("paused",reason=reason,remaining_points=len(self.window.queue.items),pose=self.stream.state.pose)
        logger.info("[AboveFlight] paused: %s; remaining=%s",reason,len(self.window.queue.items))
        logger.info("[AboveFlight] pause waits: pending_io=%.3fs hold=%.3fs",drained-started,self.clock()-drained)

    def capture_held(self, session):
        # A camera change needs a new image, not an older moving-frame result.
        self.frames.discard()
        while self.frames.job.busy:
            self.checkpoint()
            self.sleep(.01)
        self.checkpoint()
        frame = self.stopped_source.capture(budget=session.budget)
        self.last_capture_request = self.clock()
        self.latest = self.observation = frame
        return frame

    def resume(self, session, trace):
        self.stream.sample()
        self.replace_queue(session,trace)
        self.latest = None  # obtain a new frame-time queue anchor after resume
        self.observation = None
        self.last_capture_request = -float("inf")

    def replace_queue(self, session, trace):
        if self.stream.record is not None:
            session.last_executed_before_replan = self.stream.record
            trace.event("executed_before_replan",record=self.stream.record)
        self.flush_telemetry(trace)
        # Keep the stage token, cumulative requests and deadline unchanged.
        self.window.queue.clear()
        self.window.last_stream_submit = -float("inf")
        self.window.stream_retry_at = 0.0
        self.stream.begin(self.window.queue,self.window.background.stage_token)
        session.replans += 1
        trace.event("queue_cleared_for_replan",planning_calls=session.planning_calls,replans=session.replans)
        logger.info("[AboveFlight] cleared remaining queue; replan=%s cumulative_planning_calls=%s",
                    session.replans,session.planning_calls)

    def flush_telemetry(self, trace):
        samples,dropped = self.stream.drain_telemetry()
        write = getattr(trace,"control_samples",None)
        if write is not None and (samples or dropped):
            try:
                write(samples,dropped=dropped)
            except Exception as exc:
                logger.warning("[AboveDiagnostics] control samples could not be written (%s)",type(exc).__name__)

    def call(self, operation, session):
        return self.calls.call(operation,min(self.settings.model_request_timeout_s,
                              session.budget.remaining(self.clock())),self.checkpoint)

    def tick(self, parameters, session, trace):
        self.stream.sample()
        self.window.record = self.stream.record
        frame = self.frames.poll()
        if frame is not None:
            self.latest = frame
            self.observation = frame
        result = self.window.poll_stream(self.stream.state.pose,self.checkpoint)
        if result is not None:
            trace.event(result[0],request_id=result[1],detail=result[2],remaining_points=len(self.window.queue.items))
        passed = self.stream.tick(sample=False)
        if passed:
            trace.event("waypoint_passed",waypoint_ids=passed,pose=self.stream.state.pose)
        may_submit = session.binding is None or session.target_visible
        if may_submit and self.latest is not None and self.window.stream_due():
            if session.planning_calls >= self.settings.maximum_planning_calls:
                if not self.window.queue.items:
                    raise BudgetExceeded("above stage planning budget exhausted")
            else:
                self._submit(parameters,session,trace)
        # One capture pump; neither queue capacity nor model latency sets its cadence.
        if self.clock()-self.last_capture_request >= self.settings.observation_interval_s:
            self.frames.request(session.budget)
            self.last_capture_request = self.clock()
        return frame

    def _submit(self, parameters, session, trace):
        frame = self.latest
        try:
            anchor = self.stream.timeline.at(frame.timestamp_ns,self.window.background.stage_token)
        except ProtocolError:
            if self.window.queue.items or not frame.stopped:
                return
            self.window.queue.require_anchor(self.stream.state.pose,frame.vehicle_pose)
            anchor = QueueObservationAnchor(self.window.background.stage_token,self.window.queue.revision,frame.timestamp_ns,())
        planning = PlanningWindowInput(session.initial,self.window.settings.max_observation_age_s,
                                      self.settings.model_request_timeout_s)
        def count():
            session.planning_calls += 1
        try:
            submitted = self.window.submit_stream(frame,session.context(parameters),parameters.instruction,
                planning,anchor,count,self.checkpoint,trace_images=trace.planning_observation)
        except ProtocolError as exc:
            trace.event("planning_snapshot_discarded",reason=str(exc),capture_id=frame.capture_id)
            submitted = False
        if submitted:
            trace.event("planning_submitted",phase=session.phase,planning_calls=session.planning_calls,
                        capture_id=frame.capture_id,pending=len(anchor.items),destination=session.destination)
        # A frame is consumed once, including when its old queue anchor expired.
        self.latest = None

    def close(self):
        close_resources((self.window.close,self.frames.close,self.detection.close,self.calls.close,
                         self.close_stopped_source))
