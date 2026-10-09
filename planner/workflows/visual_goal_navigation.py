"""Static visual-goal observe/approach/verify workflow; dependencies are injected."""
import logging
import math
import time
from typing import Callable

import numpy as np

from planner.capabilities.targeting.association import make_reference, coherent_surface_motion
from planner.capabilities.targeting.surface_selection import SurfaceSelector
from planner.domain.execution import ExecutionBudget, StopResult
from planner.domain.pose import RelativePoseDelta, wrap_yaw_deg
from planner.domain.result import StageResult
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import (
    GoalVisibility, VisualGoalNavigationParameters, VisualGoalNavigationResult,
    VisualGoalPlanningContext, VisualGoalState, VisualGoalStatus,
)
from planner.errors import BudgetExceeded, ExecutionError, ObservationError, ProtocolError
from planner.domain.mission import MissionStage
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.ports.observation_source import ObservationSource
from planner.ports.goal_observer import GoalObserver
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.execution import ExecutionService
from planner.services.motion.base import MotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.services.trajectory_safety import TrajectorySafetyChecker
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.ports.task_trace import TaskTrace
from planner.services.trajectory_transform import relative_to_world
from .termination.visual_goal_navigation import SearchPolicy, VisualGoalExitPolicy, VisualGoalFailure
from .visual_goal_session import VisualGoalSession, model_call

logger = logging.getLogger(__name__)


class VisualGoalNavigationWorkflow:
    def __init__(self, observation_source: ObservationSource, goal_observer: GoalObserver,
                 trajectory_planner: TrajectoryPlanner, validator: TrajectoryValidator,
                 motion_planner: MotionPlanner, execution: ExecutionService,
                 geometry: DepthGeometry, completion: VisualGoalCompletion,
                 safety: TrajectorySafetyChecker, limits: VisualGoalLimits,
                 surface_selector: SurfaceSelector,
                 trace_factory: Callable[[], TaskTrace] | None = None,
                 clock=time.monotonic, sleep=time.sleep):
        self.source, self.goal_observer, self.planner = observation_source, goal_observer, trajectory_planner
        self.validator, self.motion, self.execution = validator, motion_planner, execution
        self.geometry, self.completion, self.safety, self.limits = geometry, completion, safety, limits
        self.surface_selector = surface_selector
        self.trace_factory, self.clock, self.sleep = trace_factory, clock, sleep

    def run(self, stage: MissionStage) -> StageResult[object]:
        started = self.clock()
        session = VisualGoalSession(ExecutionBudget(started+self.limits.maximum_task_duration_s),
                                    VisualGoalExitPolicy(self.limits), SearchPolicy(self.limits), started)
        trace = None
        try:
            parameters = stage.parameters
            if not isinstance(parameters, VisualGoalNavigationParameters):
                raise VisualGoalFailure(VisualGoalStatus.BLOCKED, "invalid visual goal parameters")
            self.limits.validate_standoff(parameters.standoff_m, self.geometry.maximum_m)
            if self.trace_factory:
                trace = self.trace_factory()
                trace.event("task_start", stage_id=stage.stage_id, parameters=parameters, limits=self.limits)
            status, reason = self._run(parameters, session, trace)
            stop = StopResult(True)  # Last observation verified stopped, with no intervening motion.
        except (Exception, KeyboardInterrupt) as exc:
            if isinstance(exc, VisualGoalFailure):
                status = exc.status
            elif isinstance(exc, BudgetExceeded):
                status = VisualGoalStatus.TIMEOUT
            elif isinstance(exc, ObservationError):
                status = VisualGoalStatus.OBSERVATION_ERROR
            elif isinstance(exc, ProtocolError):
                status = VisualGoalStatus.MODEL_ERROR
            elif isinstance(exc, KeyboardInterrupt):
                status = VisualGoalStatus.CANCELLED
            elif isinstance(exc, ExecutionError):
                status = VisualGoalStatus.EXECUTION_ERROR
            else:
                status = VisualGoalStatus.BLOCKED
            reason = str(exc) or status.value
            stop = getattr(exc, "stop_result", None) or self.execution.stop()
            if trace is not None and getattr(exc, "receipt", None) is not None:
                self._event(trace, "partial_execution", receipt=exc.receipt)
        distance = session.geometry.distance_m if session.geometry else None
        result = VisualGoalNavigationResult(status, reason, distance, session.policy.planning_calls,
                                            self.clock()-started, session.path_length_m,
                                            session.confirmations, stop,
                                            run_directory=str(trace.path) if trace else None)
        self._event(trace, "result", result=result)
        logger.info("[VisualGoal] %s: %s distance=%s stop_confirmed=%s", status.value, reason, distance, stop.confirmed)
        return StageResult(stage.stage_id, status is VisualGoalStatus.REACHED, result,
                           None if status is VisualGoalStatus.REACHED else f"{status.value}: {reason}")

    def _capture(self, session):
        session.require_time(self.clock)
        observation = self.source.capture(budget=session.budget)
        session.require_time(self.clock)
        if not observation.stopped:
            raise ObservationError("task observation source did not verify stopped vehicle")
        if session.initial is None:
            session.initial = observation.vehicle_pose
            rotation = np.asarray(observation.rotation_camera_to_world)
            pitch = math.degrees(math.asin(float(np.clip(-rotation[2, 0], -1, 1))))
            if abs(pitch) > self.limits.maximum_pitch_abs_deg:
                raise VisualGoalFailure(VisualGoalStatus.BLOCKED, "camera is outside level-view pitch envelope")
            session.free_balls = self.safety.begin(observation)
        return observation

    def _run(self, parameters, session, trace):
        while True:
            observation = self._capture(session)
            self._event(trace, "round_observation", capture_id=observation.capture_id,
                        pose=observation.vehicle_pose,
                        remaining_budget_s=session.budget.remaining(self.clock()))
            if not session.fresh(observation):
                session.confirmations = 0
                session.policy.unreliable_observation()
                self._wait(self.limits.confirmation_interval_s, session)
                continue
            observed_goal = model_call(lambda: self.goal_observer.observe(
                observation, parameters.target_description, session.reference,
                initial_pose=session.initial, budget=session.budget,
            ), session, self.limits, clock=self.clock)
            if trace:
                trace.observation(observation, observed_goal)
            if not session.policy.anchor_valid(observation, self.execution.current_pose(), self.clock()):
                session.confirmations = 0
                continue
            if observed_goal.status is GoalVisibility.NOT_VISIBLE:
                self._event(trace, "state", state=VisualGoalState.SEARCHING)
                session.confirmations = 0
                self._search(observation, parameters, session, trace)
                continue
            if observed_goal.status is GoalVisibility.AMBIGUOUS:
                session.confirmations = 0
                session.policy.unreliable_observation(ambiguous=True)
                continue
            desired = parameters.standoff_m if parameters.standoff_m is not None else self.limits.standoff_m
            estimates = self.geometry.estimate_candidates(observation, observed_goal, self.limits)
            selection = self.surface_selector.select(observation, observed_goal, estimates,
                                                    session.reference, session.initial, desired)
            session.surface_selection = selection
            geometry, goal = selection.geometry, selection.goal
            session.geometry = geometry
            self._event(trace, "surface_selection", selection=selection)
            if trace:
                trace.surface_selection(observation, selection)
            if selection.selected is None:
                session.confirmations = 0
                suspect = self.surface_selector.motion_evidence(observed_goal, selection)
                if suspect and coherent_surface_motion(session.reference, observed_goal.at(suspect.candidate),
                        suspect.geometry, session.displaced_surface, self.limits.association_gate_m,
                        self.limits.distance_uncertainty_margin_m,
                        matches_reference_region=suspect.candidate.matches_reference_region):
                    raise VisualGoalFailure(VisualGoalStatus.TARGET_MOVED,
                        "associated surface has coherent multi-observation world displacement; static-target premise rejected")
                session.displaced_surface = suspect.geometry.world_point if suspect else None
                session.policy.unreliable_observation()
                continue
            session.displaced_surface = None
            session.policy.unreliable = 0
            session.search.found()
            if session.reference is None:
                session.reference = make_reference(observation, goal, geometry, selection.selected.candidate.description)
            clearance = (self.safety.clearance(observation) and not self.execution.collision_state()
                         and self.completion.surface_clearance(parameters, selection))
            if not clearance:
                session.confirmations = 0
                raise VisualGoalFailure(VisualGoalStatus.SAFETY_STOP, "observed obstacle/body clearance is insufficient")
            if self.completion.candidate(parameters, observation, goal, geometry, session.initial, clearance,
                                         surface_selection=selection):
                if self.clock()-session.last_confirmation >= self.limits.confirmation_interval_s:
                    session.confirmations += 1
                    session.last_confirmation = self.clock()
                self._event(trace, "confirming", state=VisualGoalState.VERIFYING,
                            count=session.confirmations, geometry=geometry)
                if session.confirmations >= self.limits.confirmation_count:
                    return VisualGoalStatus.REACHED, "same visible target, depth arrival band and stopped observations confirmed"
                self._wait(self.limits.confirmation_interval_s, session)
                continue
            session.confirmations = 0
            if geometry.distance_m < desired-self.limits.distance_tolerance_m:
                raise VisualGoalFailure(VisualGoalStatus.SAFETY_STOP, "already too close; blind backward recovery is disabled")
            # Only post-execution approach rounds count toward distance progress.
            if session.translation_pending_progress:
                session.policy.record_progress(geometry.distance_m)
                session.translation_pending_progress = False
            context = VisualGoalPlanningContext(parameters.target_description, goal, geometry,
                VisualGoalState.APPROACHING, desired, self.limits.distance_tolerance_m,
                self.limits.maximum_round_distance_m, self.limits.maximum_round_motion_s,
                self.limits.maximum_vertical_offset_m, self.limits.reobserve_heading_change_deg,
                self.limits.clearance_radius_m)
            self._event(trace, "state", state=VisualGoalState.APPROACHING)
            self._approach(observation, context, parameters, session, trace)

    def _approach(self, observation, context, parameters, session, trace):
        for attempt in range(self.limits.safety_replan_count+1):
            relative = model_call(lambda: self.planner.plan(
                observation, parameters.instruction, round_index=session.policy.planning_calls,
                initial_pose=session.initial, previous_trajectory=session.previous,
                current_guidance=None if attempt == 0 else "Previous trajectory rejected. Shorten complete segments or output pure yaw first.",
                visual_goal_context=context, budget=session.budget,
            ), session, self.limits, is_planning=True, validate=self.validator.validate_relative, clock=self.clock)
            world = relative_to_world(relative, observation.vehicle_pose)
            self.validator.validate_world(world)
            motion = self.motion.create_motion(world)
            selection = self.safety.select(motion, observation, session.initial, session.free_balls,
                                           context.geometry.world_point,
                                           context.standoff_m-context.tolerance_m+self.limits.distance_uncertainty_margin_m,
                                           additional_standoff_points=session.surface_selection.reliable_world_points)
            self._event(trace, "plan", predicted=relative, selection=selection)
            if not session.policy.anchor_valid(observation, self.execution.current_pose(), self.clock()):
                return
            if selection.motion.segments:
                self._execute(selection.motion, session, trace)
                session.previous = RelativeTrajectory(relative.points[:len(selection.motion.segments)])
                session.free_balls = self.safety.record_completed(session.free_balls, selection)
                return
        raise VisualGoalFailure(VisualGoalStatus.BLOCKED, selection.reason)

    def _search(self, observation, parameters, session, trace):
        session.require_time(self.clock)
        heading = session.search.next_heading(observation.vehicle_pose, session.reference,
                                               parameters.target_description)
        relative = RelativeTrajectory((RelativePoseDelta(0, 0, 0, wrap_yaw_deg(heading-observation.vehicle_pose.yaw_deg)),))
        if not self.safety.clearance(observation) or self.execution.collision_state():
            raise VisualGoalFailure(VisualGoalStatus.SAFETY_STOP, "search clearance contradicted by current evidence")
        if not session.policy.anchor_valid(observation, self.execution.current_pose(), self.clock()):
            return
        self.validator.validate_relative(relative)
        world = relative_to_world(relative, observation.vehicle_pose)
        self._execute(self.motion.create_motion(world), session, trace)
        session.previous = relative
        self._event(trace, "search", heading=heading, moves=session.search.total_moves)

    def _execute(self, motion, session, trace):
        round_budget = ExecutionBudget(min(session.budget.deadline_monotonic,
            self.clock()+self.limits.maximum_round_motion_s))
        before = self.execution.current_pose()
        receipt = self.execution.execute_with_receipt(motion, budget=round_budget)
        session.policy.anchor_failures = 0
        measured = session.policy.record_motion(before, receipt.final_pose)
        session.path_length_m += receipt.measured_chord_length_m
        session.translation_pending_progress = measured >= 0.2
        self._event(trace, "executed", receipt=receipt, motion=motion)

    def _wait(self, seconds, session):
        self.sleep(min(seconds, session.budget.remaining(self.clock())))
        session.require_time(self.clock)

    @staticmethod
    def _event(trace, name, **data):
        if trace is not None:
            try:
                trace.event(name, **data)
            except Exception:
                logger.exception("Diagnostic write failed: %s", name)
