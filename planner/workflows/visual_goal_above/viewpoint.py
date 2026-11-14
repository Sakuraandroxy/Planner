"""One stopped camera adjustment; all flight displacement comes from the planner."""
import logging

from planner.capabilities.viewing.bbox_top_view import bbox_top_view
from planner.errors import ModelCallTimeout
from .observation_policy import observation_target, needs_observation

logger = logging.getLogger(__name__)


class AboveViewpoint:
    def __init__(self, flight, view, perception, settings):
        self.flight, self.view, self.perception, self.settings = flight, view, perception, settings

    def observe(self, parameters, session, trace):
        f = self.flight
        frame = f.capture_held(session)
        try:
            result = f.call(self.perception.operation(frame,parameters,session,
                min(self.settings.model_request_timeout_s,session.budget.remaining(f.clock()))),session)
        except ModelCallTimeout:
            logger.warning("[AbovePerception] stopped detection timed out; retaining stage geometry")
            result = None
        if result is None:
            # A failed request says nothing about the previously measured roof.
            session.target_visible = False
            session.last_detection_capture_id = frame.capture_id
        else:
            self.perception.accept(result,session,trace)
        session.last_detection = f.clock()
        return frame

    def recover(self, parameters, session, trace):
        self.view.recover_visible()
        trace.event("camera_view_recovered")
        frame = self.observe(parameters,session,trace)
        if not session.target_visible and observation_target(session) is not None:
            self.view.look_at(observation_target(session))
            frame = self.observe(parameters,session,trace)
        return frame

    def run(self, parameters, session, trace, *, first=False,
            reason="change camera view and observe target"):
        f = self.flight
        if not first:
            f.pause(session,trace,reason)
        frame = self.observe(parameters,session,trace)
        if session.target_visible:
            self.view.remember_visible()
        if needs_observation(session,frame):
            target = observation_target(session)
            if session.target_visible and session.bbox_frame is not None and not session.regions:
                cue = bbox_top_view(session.bbox_frame,session.bbox)
                self.view.look_direction(cue.direction_world)
                trace.event("camera_bbox_view",bbox=session.bbox,
                    edge_elevations_deg=cue.edge_elevations_deg,clipped_top=cue.clipped_top,
                    top_detection_enabled=session.top_detection_enabled,capture_id=frame.capture_id)
            elif target is not None:
                self.view.look_at(target)
                trace.event("camera_target_view",target_world=target)
            elif self.view.last_visible is not None:
                self.view.recover_visible()
                trace.event("camera_view_recovered")
            frame = self.observe(parameters,session,trace)
            session.top_view_needed = False
            if not session.target_visible and self.view.last_visible is not None:
                frame = self.recover(parameters,session,trace)
            if session.target_visible:
                self.view.remember_visible()
        session.last_view = f.clock()
        if first:
            f.latest = frame
        else:
            # A deliberate observation is a new decision point, even when
            # both old and new roof destinations are None.
            f.resume(session,trace)
