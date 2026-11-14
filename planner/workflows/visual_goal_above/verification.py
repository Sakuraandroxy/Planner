"""Pause, inspect and either resume or replan; false is not a stage failure."""
from planner.capabilities.completion.visual_goal_above import completion_request, locally_complete
from planner.services.observation_snapshot import freeze_observation
from planner.errors import ModelCallTimeout
from planner.domain.visual_goal_above import AboveDetectionMode
import logging


class AboveVerification:
    def __init__(self, flight, view, perception, checker, settings):
        self.flight,self.view,self.perception,self.checker,self.settings = flight,view,perception,checker,settings

    def run(self, parameters, session, trace):
        flight = self.flight
        flight.pause(session,trace,"above completion observation")
        # Stay at the actual arrival pose. No hidden move toward an old goal point.
        self.view.down()
        frame = flight.capture_held(session)
        try:
            result = flight.call(self.perception.operation(frame,parameters,session,
                min(self.settings.model_request_timeout_s,session.budget.remaining(flight.clock())),
                mode=AboveDetectionMode.TARGET_AND_TOP),session)
        except ModelCallTimeout:
            logging.getLogger(__name__).warning("[AbovePerception] downward detection timed out; using measured depth to update prior geometry")
            result = None
        self.perception.accept(result,session,trace,frame=frame)
        session.last_detection = flight.clock()
        request = completion_request(parameters,freeze_observation(frame),session.regions,session.height,
            self.settings.height_tolerance_m,flight.holding.observable,session.reference_rgb)
        verdict = False
        if locally_complete(request):
            verdict = flight.call(lambda: self.checker.check(request,
                timeout_s=min(self.settings.model_request_timeout_s,session.budget.remaining(flight.clock()))),session)
        write = getattr(trace,"completion_observation",None)
        if write is not None:
            try:
                write(request,verdict)
            except Exception:
                logging.getLogger(__name__).warning("[AboveDiagnostics] completion image write failed")
        trace.event("completion_check",projection_inside=request.projection_inside,
                    measured_height_m=request.measured_height_m,height_condition_met=request.height_condition_met,
                    hold_confirmed=request.hold_confirmed,verdict=verdict,
                    pose=frame.vehicle_pose,body_projection_world=request.body_projection_world,
                    body_projection_image=request.body_projection_image,capture_id=frame.capture_id,
                    height_is_minimum=parameters.height_above_m is None,
                    camera_rotation=frame.rotation_camera_to_world)
        logging.getLogger(__name__).info("[AboveCompletion] verdict=%s projection_inside=%s height_above_m=%s hold_confirmed=%s",
            verdict,request.projection_inside,request.measured_height_m,request.hold_confirmed)
        if verdict:
            session.last_check = flight.clock()
            flight.window.complete()
            return True
        flight.resume(session,trace)
        session.last_check = flight.clock()
        return False
