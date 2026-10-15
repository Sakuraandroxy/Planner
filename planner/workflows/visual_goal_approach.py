"""Direct model trajectory execution and immutable refill context preparation."""
import math
from dataclasses import replace
from functools import partial
from planner.services.observation_snapshot import freeze_observation

from planner.domain.visual_goal import VisualGoalPlanningContext, VisualGoalState, GoalVisibility
from planner.errors import ExecutionError
from planner.services.trajectory_transform import relative_to_world, world_to_relative
from .visual_goal_session import model_call


class VisualGoalApproachPhase:
    def __init__(self, planner, validator, motion, execution, limits, flight, capture, clock, event):
        self.planner, self.validator, self.motion = planner, validator, motion
        self.execution, self.limits, self.flight = execution, limits, flight
        self.capture, self.clock, self._event = capture, clock, event

    def run(self, observation, context, parameters, session, trace):
        if session.window is not None:
            world = session.window.next_world(observation, context, parameters.instruction, session.window_input(),
                session.policy.planning_call, lambda: self.checkpoint(session))
            relative = world_to_relative(world)
        else:
            operation = partial(self.planner.plan, freeze_observation(observation), parameters.instruction,
                round_index=session.policy.planning_calls+1, initial_pose=session.initial,
                previous_trajectory=session.previous, previous_trajectory_start=session.previous_start,
                current_guidance=None, task_context=context, budget=session.budget)
            relative = model_call(operation, session, self.limits, is_planning=True,
                validate=self.validator.validate_relative, clock=self.clock)
            world = relative_to_world(relative, observation.vehicle_pose)
        self.validator.validate_world(world)
        if not session.policy.anchor_valid(observation, self.execution.current_pose(), self.clock()):
            return
        motion = self.motion.create_motion(world)
        self._event(trace, "plan", predicted=relative, motion=motion, execution_policy="direct_model_trajectory")
        self.flight.execute(motion, session, trace, enable_completion_pause=True,
            prefetch=lambda: self.prefetch(observation, context.goal, parameters, session))

    def prefetch(self, observation, goal, parameters, session):
        if session.policy.planning_calls >= self.limits.maximum_planning_rounds:
            # Exhausted refill budget must not interrupt already queued actions.
            return
        radius = self.flight.trigger_rules.radius_m
        context = VisualGoalPlanningContext(parameters.target_description, goal, session.geometry,
            VisualGoalState.APPROACHING, radius, self.limits.distance_uncertainty_margin_m,
            self.limits.maximum_vertical_offset_m, True, session.target_capture_id,
            session.target_captured_monotonic, parameters.keep_target_in_view)
        current_pose = self.execution.current_pose()
        if (session.window.record is not None or current_pose != observation.vehicle_pose or
                self.clock()-observation.captured_monotonic > self.limits.maximum_observation_age_s):
            observation = self.capture(session)
            point = session.geometry.world_point
            pose = observation.vehicle_pose
            geometry = replace(session.geometry,
                distance_m=math.dist((pose.x, pose.y, pose.z), point),
                reason="distance to last observed bbox-center position; not a fresh depth measurement")
            context = replace(context, goal_observation_is_current=False, geometry=geometry,
                goal=replace(goal, status=GoalVisibility.AMBIGUOUS, bbox=None, surface_point=None,
                    evidence="previously bound target identity; current-image correspondence not supplied"))
        session.window.prefetch(observation, context, parameters.instruction, session.window_input(),
            session.policy.planning_call, session.pending_motion, lambda: self.checkpoint(session))

    def checkpoint(self, session):
        session.require_time(self.clock)
        if self.execution.collision_state():
            raise ExecutionError("collision during window model wait")

