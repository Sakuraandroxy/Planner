"""Hybrid continuous flight and stopped observation for one static above target."""
import logging

from planner.domain.execution import ExecutionBudget
from planner.domain.result import StageResult, StageSummary, CompletionBasis, ResultMeasurement
from planner.domain.visual_goal_above import VisualGoalAboveParameters
from planner.errors import BudgetExceeded, ExecutionError, ObservationError, ProtocolError, SimulatorConnectionError, ModelCallTimeout
from .session import AboveSession
from .verification import AboveVerification
from .viewpoint import AboveViewpoint
from .observation_policy import needs_observation, record_arrival_state
from planner.capabilities.targeting.top_geometry import top_height

logger = logging.getLogger(__name__)


class VisualGoalAboveWorkflow:
    def __init__(self, flight, view, perception, checker, settings, trace_factory):
        self.flight,self.view,self.perception,self.settings = flight,view,perception,settings
        self.trace_factory = trace_factory
        self.verification = AboveVerification(flight,view,perception,checker,settings)
        self.viewpoint = AboveViewpoint(flight,view,perception,settings)

    def prepare(self):
        if self.flight.pending():
            raise ExecutionError("a previous above-stage request is still running")

    def _run(self, parameters, session, trace):
        f = self.flight
        self.viewpoint.run(parameters,session,trace,first=True)
        while True:
            started = f.clock()
            f.checkpoint()
            try:
                result = f.detection.poll()
            except ModelCallTimeout:
                logger.warning("[AbovePerception] moving detection timed out; retaining stage geometry")
                result = None
            if result is not None and self.perception.accept(result,session,trace):
                f.pause(session,trace,"top geometry changed; replan remaining motion")
                d = session.destination
                if d is not None:
                    self.view.look_at((d.x,d.y,top_height(session.known_regions,d.x,d.y)))
                f.resume(session,trace)
            pose = f.stream.state.pose
            record_arrival_state(session,pose,self.settings,f.clock(),trace)
            if session.binding is not None and not session.target_visible:
                # Cancel old motion immediately; retry at the existing detection
                # cadence while owner-side position hold remains active.
                if (f.window.queue.items or f.window.background.job is not None or
                        not f.stream.holding or
                        f.clock()-session.last_detection >= self.settings.detection_interval_s):
                    self.viewpoint.run(parameters,session,trace,reason="recover bound target")
                if not session.target_visible:
                    f.sleep(max(.001,1/f.stream.limits.control_hz-(f.clock()-started)))
                    continue
                pose = f.stream.state.pose
            if session.should_verify(pose,self.settings,f.clock()):
                if self.verification.run(parameters,session,trace):
                    return
            elif (f.clock()-session.last_view >= self.settings.view_checkpoint_interval_s and
                  needs_observation(session,f.observation)):
                self.viewpoint.run(parameters,session,trace)
            f.tick(parameters,session,trace)
            frame = f.observation
            if (frame is not None and frame.capture_id != session.last_detection_capture_id and
                    f.clock()-session.last_detection >= self.settings.detection_interval_s):
                if not f.detection.busy and f.detection.future is None and not f.calls.has_pending_call:
                    timeout = min(self.settings.model_request_timeout_s,session.budget.remaining(f.clock()))
                    f.detection.start(self.perception.operation(frame,parameters,session,timeout),timeout)
                    session.last_detection = f.clock()
                    session.last_detection_capture_id = frame.capture_id
            f.sleep(max(.001,1/f.stream.limits.control_hz-(f.clock()-started)))

    def run(self, stage):
        f = self.flight
        began, leased, success = f.clock(),False,False
        trace = self.trace_factory()
        status,reason,session = "execution_error","stage not started",None
        cleanup_errors = []
        try:
            self.prepare()
            if not isinstance(stage.parameters,VisualGoalAboveParameters):
                raise ProtocolError("above stage has invalid parameters")
            f.execution.acquire_flight(self)
            leased = True
            deadline = began+min(self.settings.maximum_duration_s,f.window.settings.max_task_duration_s)
            f.execution.set_deadline(deadline)
            f.stop()
            session = AboveSession(f.execution.vehicle.current_pose(),ExecutionBudget(deadline),
                stage.parameters.height_above_m or self.settings.default_height_above_m,
                height_is_minimum=stage.parameters.height_above_m is None,
                height_tolerance_m=self.settings.height_tolerance_m)
            f.holding.wait(budget=session.budget,checkpoint=f.checkpoint)
            self.view.begin()
            trace.event("stage_start",stage_id=stage.stage_id,parameters=stage.parameters,
                        initial=session.initial,deadline_monotonic=deadline,original_camera=self.view.original)
            f.begin(session)
            self._run(stage.parameters,session,trace)
            status,reason,success = "reached","above-target completion confirmed by model and local measurements",True
        except KeyboardInterrupt:
            status,reason = "cancelled","interrupted by user"
            raise
        except Exception as exc:
            if isinstance(exc,BudgetExceeded):
                status = "timeout"
            elif isinstance(exc,ProtocolError):
                status = "model_error"
            elif isinstance(exc,ObservationError):
                status = "observation_error"
            elif isinstance(exc,SimulatorConnectionError):
                status = "connection_error"
            elif isinstance(exc,ExecutionError) and str(exc) == "flight stop requested":
                status = "cancelled"
            reason = str(exc)
            logger.error("[VisualGoalAbove %s] %s: %s",stage.stage_id,status,reason,exc_info=True)
        finally:
            if leased:
                # Cleanup must work after a stop/deadline signal too: stop and
                # restore do not call the cancelled task's checkpoint.
                for name,action in (("stop",f.stop),
                                    ("discard planning",f.window.background.end),
                                    ("discard capture",f.frames.discard),
                                    ("discard detection",f.detection.discard),
                                    ("clear queue",f.window.queue.clear),
                                    ("restore camera",self.view.restore)):
                    try:
                        action()
                    except Exception as exc:
                        cleanup_errors.append(f"{name}: {exc}")
                f.flush_telemetry(trace)
                f.window.end()
                f.execution.release_flight(self,pending_cleanup=f.pending)
            if cleanup_errors:
                trace.event("cleanup_error",primary_status=status,primary_reason=reason,
                            errors=cleanup_errors)
                if success:
                    status,reason = "cleanup_error","; ".join(cleanup_errors)
                else:
                    reason += "; cleanup: "+"; ".join(cleanup_errors)
                success = False
            trace.event("stage_end",status=status,reason=reason,success=success,
                        planning_calls=session.planning_calls if session else 0,
                        replans=session.replans if session else 0)
            close_trace = getattr(trace,"close",None)
            if close_trace is not None:
                try:
                    close_trace()
                except Exception:
                    logger.warning("[AboveDiagnostics] diagnostic writer could not close")
        actual = f.stream.state.pose if f.stream.state is not None else None
        summary = StageSummary(status,reason,CompletionBasis.MODEL_WITH_LOCAL_CHECKS if success else CompletionBasis.NOT_VERIFIED,
            success,str(trace.path),(ResultMeasurement("elapsed",f.clock()-began,"s"),
            ResultMeasurement("planning_calls",session.planning_calls if session else 0),
            ResultMeasurement("replans",session.replans if session else 0)),actual)
        logger.info("[VisualGoalAbove %s] %s: %s",stage.stage_id,status,reason)
        return StageResult(stage.stage_id,success,error=None if success else reason,summary=summary)

    def close(self):
        self.flight.close()
