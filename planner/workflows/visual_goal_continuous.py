"""Visual-task orchestration over shared streaming, capture and planning services."""
from functools import partial
import math

from planner.capabilities.targeting.visual_goal_refresh import VisualGoalRefresh
from planner.capabilities.targeting.association import remembered_geometry, remembered_goal
from planner.domain.streaming import QueueObservationAnchor
from planner.domain.trajectory import MotionSegment, MotionTrajectory
from planner.domain.visual_goal import GoalVisibility, VisualGoalPlanningContext, VisualGoalState, VisualGoalStatus
from planner.errors import ProtocolError, ExecutionError, ObservationError
from planner.services.continuous_execution import ContinuousExecution
from planner.services.moving_observations import MovingObservations
from planner.services.observation_snapshot import freeze_observation, freeze_reference
from planner.services.target_coordinates import point_in_horizontal_body
from .visual_goal_session import model_call
from .termination.visual_goal_navigation import VisualGoalFailure


class VisualGoalContinuous:
    def __init__(self, execution, moving_source, limits, motion_limits, speed_mps, observer, geometry,
                 completion, flight, capture_stopped, search, clock, sleep, event, waypoint_timeout_s=60., *,
                 target_updates=None):
        self.execution, self.limits, self.completion = execution, limits, completion
        self.flight, self.capture_stopped, self.search = flight, capture_stopped, search
        self.clock, self.sleep, self.event = clock, sleep, event
        self.stream = ContinuousExecution(execution, execution.vehicle, motion_limits, speed_mps, clock, waypoint_timeout_s)
        self.frames = MovingObservations(moving_source, clock, limits.model_request_timeout_s,
                                         limits.maximum_anchor_refresh_failures+1)
        self.targets = target_updates if target_updates is not None else VisualGoalRefresh(observer, geometry, limits, clock, event)
        self.owns_targets = target_updates is None

    def prepare(self):
        if not all(callable(getattr(self.execution.vehicle, name, None))
                   for name in ('stream_state', 'stream_velocity')):
            raise ExecutionError("continuous execution requires streaming vehicle capability")
        if self.frames.job.busy or self.targets.job.busy:
            raise ExecutionError("previous capture or target request remains quarantined")

    def has_pending_calls(self):
        return self.frames.job.busy or self.targets.job.busy

    def close(self):
        self.frames.close()
        if self.owns_targets:
            self.targets.close()

    def _observe_stopped(self, parameters, session, trace):
        observation = self.capture_stopped(session)
        if not session.fresh(observation):
            session.policy.unreliable_observation()
            return None
        operation = partial(self.targets.observer.observe, freeze_observation(observation),
            parameters.target_description, freeze_reference(session.reference),
            initial_pose=session.initial, budget=session.budget, instruction=parameters.instruction)
        goal = model_call(operation, session, self.limits, clock=self.clock)
        if goal.status is GoalVisibility.NOT_VISIBLE and not session.pending_motion.segments:
            self.search(observation, parameters, session, trace)
            return None
        if not self.targets.accept(observation, goal, session, trace):
            session.policy.unreliable_observation()
            return None
        session.policy.unreliable = 0
        return observation

    def _sync_remaining(self, session):
        items = session.window.queue.items
        segments = []
        for i, item in enumerate(items):
            start = self.stream.state.pose if i == 0 else item.start
            yaw = self.stream.goal_yaw-self.stream.yaw if i == 0 and self.stream.goal_yaw is not None else item.yaw_delta_deg
            segments.append(MotionSegment(start, item.end, 'continuous', target_speed_mps=self.stream.follower.speed,
                                          yaw_delta_deg=yaw))
        session.pending_motion = MotionTrajectory(tuple(segments))
        session.previous = self.stream.record.actions
        session.previous_start = self.stream.record.start
        session.path_length_m = self.stream.path_length_m

    def _pause_reason(self, session, pose):
        if session.trigger.should_pause(pose):
            return session.trigger.last_reason
        return None

    def _check_pause(self, parameters, session, trace):
        window = session.window
        window.background.discard()
        self.frames.discard()
        self.targets.job.discard()
        memory = session.target_memory  # one immutable estimate for this entire check
        session.confirmations = 0
        while True:
            observation = self.capture_stopped(session)
            if not session.fresh(observation):
                session.policy.unreliable_observation()
                continue
            self.stream.sample()
            self._sync_remaining(session)
            goal = remembered_goal(parameters.target_description)
            geometry = remembered_geometry(memory, observation.vehicle_pose)
            session.geometry = geometry
            if not self.completion.candidate(parameters, observation, goal, geometry, session.initial,
                                             radius_m=self.flight.trigger_rules.radius_m, target_memory=memory):
                session.trigger.checked_at(observation.vehicle_pose)
                self.stream.resume()
                return observation, False
            session.confirmations += 1
            self.event(trace, 'confirming', count=session.confirmations, geometry=geometry)
            if session.confirmations < self.limits.confirmation_count:
                self.sleep(self.limits.confirmation_interval_s)
                continue
            verdict = self.flight.verify(parameters, observation, goal, session, trace, target_memory=memory)
            if verdict is None:
                session.confirmations = 0
                continue
            session.trigger.checked_at(observation.vehicle_pose)
            if verdict:
                self.event(trace, 'completion_confirmed', remaining_points=len(window.queue.items))
                session.completion_confirmed = True
                session.pending_motion = MotionTrajectory(())
                window.complete()
                return observation, True
            self.stream.resume()
            return observation, False

    def _submit(self, observation, parameters, session, trace):
        window = session.window
        memory = session.target_memory
        if memory.captured_monotonic > observation.captured_monotonic:
            return False
        try:
            anchor = self.stream.timeline.at(observation.timestamp_ns, window.background.stage_token)
        except ProtocolError:
            # Initial/stopped frames precede the first streaming sample. They
            # may anchor only an empty queue after a measured stability check.
            if not observation.stopped or window.queue.items:
                return False
            window.queue.require_anchor(self.stream.state.pose, observation.vehicle_pose)
            anchor = QueueObservationAnchor(window.background.stage_token, window.queue.revision,
                                            observation.timestamp_ns, ())
        geometry = remembered_geometry(memory, observation.vehicle_pose)
        context = VisualGoalPlanningContext(parameters.target_description, remembered_goal(parameters.target_description), geometry,
            VisualGoalState.APPROACHING, self.flight.trigger_rules.radius_m,
            self.limits.distance_uncertainty_margin_m, self.limits.maximum_vertical_offset_m,
            observation.capture_id == memory.capture_id,
            memory.capture_id, memory.captured_monotonic, parameters.keep_target_in_view)
        previous_submit = window.last_stream_submit
        try:
            submitted = window.submit_stream(observation, context, parameters.instruction, session.window_input(),
                anchor, session.policy.planning_call, lambda: self.execution.checkpoint(),
                trace_images=getattr(trace, 'planning_observation', None))
        except ProtocolError as exc:
            self.event(trace, 'planning_snapshot_discarded', reason=str(exc), capture_id=observation.capture_id)
            return False
        if submitted:
            snapshot = window.background.job[0]
            self.event(trace, 'planning_submitted', request_id=snapshot.request_id,
                capture_id=observation.capture_id, timestamp_ns=observation.timestamp_ns,
                pending=snapshot.pending, pending_ids=snapshot.pending_ids, tail=snapshot.tail,
                target_capture_id=context.target_capture_id, target_world_point=context.geometry.world_point,
                pose=observation.vehicle_pose, planning_interval_s=window.settings.planning_interval_s,
                target_relative_to_observation_body_m=point_in_horizontal_body(context.geometry.world_point, observation.vehicle_pose),
                target_relative_to_pending_tail_body_m=point_in_horizontal_body(context.geometry.world_point, snapshot.tail),
                target_age_at_observation_s=observation.captured_monotonic-context.target_captured_monotonic,
                camera_position=observation.camera_position_world,
                camera_rotation=observation.rotation_camera_to_world,
                rgb_intrinsics=observation.intrinsics, depth_intrinsics=observation.depth_intrinsics)
            self.event(trace, 'planning_schedule', request_id=snapshot.request_id,
                submitted_monotonic=window.last_stream_submit,
                elapsed_since_previous_s=(window.last_stream_submit-previous_submit if math.isfinite(previous_submit) else None),
                frozen_pending_count=len(snapshot.pending.points), live_queue_count=len(window.queue.items))
        return submitted

    def run(self, parameters, session, trace):
        self.frames.begin()
        self.targets.begin()
        observation = None
        while observation is None:
            observation = self._observe_stopped(parameters, session, trace)
        self.stream.begin(session.window.queue, session.window.background.stage_token)
        used_capture = None
        queue_was_empty = True
        try:
            while True:
                tick_started = self.clock()
                session.require_time(self.clock)
                self.stream.sample()
                window = session.window
                window.record = self.stream.record
                frame = self.frames.poll()
                if frame is not None:
                    try:
                        self.targets.geometry.validate(frame)
                    except (ValueError, TypeError) as exc:
                        raise ObservationError('moving RGB/depth geometry is invalid') from exc
                    observation = frame
                    self.event(trace, 'capture_finished', capture_id=frame.capture_id, timestamp_ns=frame.timestamp_ns)
                    self.targets.submit(frame, parameters, session, trace)
                self.targets.poll(session, trace)
                result = window.poll_stream(self.stream.state.pose, lambda: self.execution.checkpoint())
                if result is not None:
                    self.event(trace, result[0], request_id=result[1], detail=result[2],
                               queue_length=len(window.queue.items))
                consumed = self.stream.tick(lambda pose: self._pause_reason(session, pose), sample=False)
                self._sync_remaining(session)
                if consumed:
                    self.event(trace, 'waypoint_passed', waypoint_ids=consumed, pose=self.stream.state.pose,
                        velocity=self.stream.state.velocity_world, queue_length=len(window.queue.items))
                if self.stream.paused is not None:
                    self.event(trace, 'pause_requested', reason=self.stream.paused.reason,
                               remaining_points=len(window.queue.items), pose=self.stream.state.pose)
                    observation, completed = self._check_pause(parameters, session, trace)
                    used_capture = None
                    if completed:
                        return VisualGoalStatus.REACHED, 'semantic completion confirmed with remembered target and stopped observations'
                empty = not window.queue.items
                if empty and not queue_was_empty:
                    self.event(trace, 'queue_exhausted', request_inflight=window.background.job is not None)
                queue_was_empty = empty
                if session.policy.planning_calls >= self.limits.maximum_planning_rounds:
                    if empty and window.background.job is None:
                        raise VisualGoalFailure(VisualGoalStatus.TIMEOUT, 'planning budget exhausted after queue drained')
                else:
                    due = window.stream_due()
                    if due and used_capture != observation.capture_id:
                        if self._submit(observation, parameters, session, trace):
                            used_capture = observation.capture_id
                    refresh_due = (not self.targets.job.busy and self.targets.job.future is None and
                        self.clock()-self.targets.last_submit >= self.limits.target_refresh_interval_s)
                    if (due or refresh_due) and self.frames.job.future is None and not self.frames.job.busy:
                        self.frames.request(session.budget)
                        self.event(trace, 'capture_started')
                self.sleep(max(.001, 1/self.stream.limits.control_hz-(self.clock()-tick_started)))
        finally:
            self.frames.discard()
            self.targets.job.discard()
            if self.stream.record is not None:
                self._sync_remaining(session)
