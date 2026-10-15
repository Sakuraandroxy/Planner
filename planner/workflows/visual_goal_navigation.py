"""Static visual-goal observe/approach/verify workflow; dependencies are injected."""
import logging
import math
import time
from typing import Callable
from functools import partial
from dataclasses import replace

import numpy as np

from planner.capabilities.targeting.association import make_reference
from planner.domain.execution import ExecutionBudget, StopResult
from planner.domain.sliding_window import SlidingWindowSettings
from planner.domain.motion import MotionLimits
from planner.domain.pose import RelativePoseDelta, wrap_yaw_deg
from planner.domain.result import StageResult, StageSummary, CompletionBasis, ResultMeasurement
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.trajectory import MotionTrajectory
from planner.domain.visual_goal_completion import (
    VisualGoalTriggerRules, VisualGoalTargetSnapshot, VisualGoalCompletionInput,
)
from planner.capabilities.completion.visual_goal_trigger import VisualGoalRadiusTrigger
from planner.capabilities.completion.visual_goal_verification import VisualGoalVerification
from planner.ports.task_completion import TaskCompletionChecker
from planner.domain.visual_goal import (
    GoalVisibility, VisualGoalNavigationParameters, VisualGoalNavigationResult,
    VisualGoalPlanningContext, VisualGoalState, VisualGoalStatus,
)
from planner.errors import BudgetExceeded, ExecutionError, ObservationError, ProtocolError, ModelCallTimeout
from planner.domain.mission import MissionStage
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.ports.observation_source import ObservationSource
from planner.ports.collision_check import CollisionCheck
from planner.ports.goal_observer import GoalObserver
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.execution import ExecutionService
from planner.services.motion.base import MotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.ports.task_trace import TaskTrace
from planner.services.trajectory_transform import relative_to_world
from planner.services.model_calls import ModelCallService
from planner.services.resource_cleanup import close_resources
from planner.services.observation_snapshot import freeze_observation, freeze_reference
from .termination.visual_goal_navigation import SearchPolicy, VisualGoalExitPolicy, VisualGoalFailure
from .visual_goal_session import VisualGoalSession, model_call
from .visual_goal_flight import VisualGoalFlightPhase
from planner.services.planning_window import PlanningWindow
from .visual_goal_approach import VisualGoalApproachPhase

logger = logging.getLogger(__name__)


class VisualGoalNavigationWorkflow:
    def __init__(self, observation_source: ObservationSource, goal_observer: GoalObserver,
                 trajectory_planner: TrajectoryPlanner[VisualGoalPlanningContext], validator: TrajectoryValidator,
                 motion_planner: MotionPlanner, execution: ExecutionService,
                 geometry: DepthGeometry, completion: VisualGoalCompletion,
                 limits: VisualGoalLimits,
                 trace_factory: Callable[[], TaskTrace] | None = None,
                 clock=time.monotonic, sleep=time.sleep, *,
                 completion_checker: TaskCompletionChecker[VisualGoalCompletionInput] | None = None,
                 trigger_rules: VisualGoalTriggerRules = VisualGoalTriggerRules(),
                 sliding_window: SlidingWindowSettings = SlidingWindowSettings(),
                 motion_limits: MotionLimits = MotionLimits(),
                 collision_check: CollisionCheck | None = None):
        self.source, self.goal_observer, self.planner = observation_source, goal_observer, trajectory_planner
        self.validator, self.motion, self.execution = validator, motion_planner, execution
        self.geometry, self.completion, self.limits = geometry, completion, limits
        self.collision_check = collision_check
        self.trace_factory, self.clock, self.sleep = trace_factory, clock, sleep
        self.trigger_rules = trigger_rules
        self.model_calls = ModelCallService(clock)
        self._closed = False
        self.window = (PlanningWindow(trajectory_planner, validator, sliding_window, motion_limits, clock)
                       if sliding_window.enabled else None)
        self.verification = VisualGoalVerification(completion_checker, clock) if completion_checker is not None else None
        self.flight = VisualGoalFlightPhase(execution, motion_planner, validator, limits,
            trigger_rules, self.verification, clock, self._event)
        self.approach = VisualGoalApproachPhase(trajectory_planner, validator, motion_planner,
            execution, limits, self.flight, self._capture, clock, self._event)

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("visual goal workflow is closed")
        self.model_calls.require_available()
        if self.verification is None:
            raise ProtocolError("visual-goal navigation requires an independent completion checker")
        self.verification.calls.require_available()
        if self.window is not None:
            self.window.worker.require_available()

    def run(self, stage: MissionStage) -> StageResult[object]:
        started = self.clock()
        session = VisualGoalSession(ExecutionBudget(started+self.limits.maximum_task_duration_s),
                                    VisualGoalExitPolicy(self.limits), SearchPolicy(self.limits), started,
                                    window=self.window, model_calls=self.model_calls)
        session.checkpoint = lambda: self.approach.checkpoint(session)
        trace = None
        leased = False
        try:
            self.prepare()
            self.execution.acquire_flight(self, collision_check=self.collision_check)
            leased = True
            self.execution.set_deadline(session.budget.deadline_monotonic)
            logger.info("[VisualGoal %s] collision_check_enabled=%s", stage.stage_id,
                        self.collision_check is not None)
            if self.window is not None:
                self.window.begin(deadline_monotonic=session.budget.deadline_monotonic,
                                  maximum_planning_rounds=session.policy.limits.maximum_planning_rounds)
            session.stage_id = stage.stage_id
            session.trigger = VisualGoalRadiusTrigger(self.trigger_rules, self.limits.maximum_observation_age_s, self.clock)
            parameters = stage.parameters
            if not isinstance(parameters, VisualGoalNavigationParameters):
                raise VisualGoalFailure(VisualGoalStatus.BLOCKED, "invalid visual goal parameters")
            self.limits.validate_arrival_radius(self.trigger_rules.radius_m, self.geometry.maximum_m)
            if self.trace_factory:
                trace = self.trace_factory()
                trace.event("task_start", stage_id=stage.stage_id, parameters=parameters, limits=self.limits,
                            arrival_radius_m=self.trigger_rules.radius_m,
                            collision_check_enabled=self.collision_check is not None)
            status, reason = self._run(parameters, session, trace)
            stop = StopResult(True)  # Last observation verified stopped, with no intervening motion.
        except (Exception, KeyboardInterrupt) as exc:
            if isinstance(exc, VisualGoalFailure):
                status = exc.status
            elif isinstance(exc, (BudgetExceeded, ModelCallTimeout)):
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
            stop = getattr(exc, "stop_result", None) or self.execution.stop_if_owner(self)
            if getattr(exc, "receipt", None) is not None:
                session.pending_motion = exc.receipt.remaining
            if trace is not None and getattr(exc, "receipt", None) is not None:
                self._event(trace, "partial_execution", receipt=exc.receipt)
        finally:
            if leased:
                if session.window is not None:
                    session.window.end()
                self.execution.release_flight(self, self.has_pending_calls)
        distance = session.geometry.distance_m if session.geometry else None
        result = VisualGoalNavigationResult(status, reason, distance, session.policy.planning_calls,
                                            self.clock()-started, session.path_length_m,
                                            session.confirmations, stop,
                                            run_directory=str(trace.path) if trace else None,
                                            evidence_source="model assessment with local depth/association/stopped checks",
                                            completion_checks=session.completion_checks,
                                            completion_confirmed=session.completion_confirmed,
                                            remaining_motion=session.pending_motion)
        self._event(trace, "result", result=result)
        logger.info("[VisualGoal] %s: %s distance=%s stop_confirmed=%s", status.value, reason, distance, stop.confirmed)
        success = status is VisualGoalStatus.REACHED and session.completion_confirmed
        summary = StageSummary(status.value, reason,
            CompletionBasis.MODEL_WITH_LOCAL_CHECKS if success else CompletionBasis.NOT_VERIFIED,
            success, result.run_directory, (ResultMeasurement("distance", distance, "m"),
                ResultMeasurement("elapsed", result.elapsed_s, "s"),
                ResultMeasurement("planning_calls", result.planning_calls),
                ResultMeasurement("stop_confirmed", stop.confirmed)))
        return StageResult(stage.stage_id, success, result,
                           None if success else f"{status.value}: {reason}", summary)

    def _capture(self, session):
        session.require_time(self.clock)
        if session.window is not None and session.paused_for_completion:
            session.window.pause_for_completion(lambda: self.approach.checkpoint(session))
            session.paused_for_completion = False
        observation = self.source.capture(budget=session.budget)
        session.require_time(self.clock)
        if not observation.stopped:
            raise ObservationError("task observation source did not verify stopped vehicle")
        self.geometry.validate(observation)
        if session.initial is None:
            session.initial = observation.vehicle_pose
            rotation = np.asarray(observation.rotation_camera_to_world)
            pitch = math.degrees(math.asin(float(np.clip(-rotation[2, 0], -1, 1))))
            if abs(pitch) > self.limits.maximum_pitch_abs_deg:
                raise VisualGoalFailure(VisualGoalStatus.BLOCKED, "camera is outside level-view pitch envelope")
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
            operation = partial(self.goal_observer.observe, freeze_observation(observation),
                parameters.target_description, freeze_reference(session.reference),
                initial_pose=session.initial, budget=session.budget, instruction=parameters.instruction)
            observed_goal = model_call(operation, session, self.limits, clock=self.clock)
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
            goal = observed_goal
            geometry = self.geometry.estimate(observation, goal)
            session.geometry = geometry
            self._event(trace, "target_geometry", geometry=geometry, point_source="current_bbox_center")
            if trace:
                trace.target_geometry(observation, goal, geometry)
            if not geometry.reliable or not goal.same_target:
                session.confirmations = 0
                session.policy.unreliable_observation()
                continue
            session.policy.unreliable = 0
            session.search.found()
            if session.reference is None:
                session.reference = make_reference(observation, goal, geometry)
            else:
                # Preserve initial appearance; refresh only the latest position estimate.
                session.reference = replace(session.reference, world_point=geometry.world_point)
            session.trigger.update(VisualGoalTargetSnapshot(session.stage_id, geometry.world_point,
                                                           observation.captured_monotonic))
            if session.trigger.last_pause_pose is None and geometry.distance_m <= self.trigger_rules.radius_m:
                session.trigger.checked_at(observation.vehicle_pose)
            if (geometry.distance_m <= self.trigger_rules.radius_m and
                    self.completion.candidate(parameters, observation, goal, geometry, session.initial,
                                              radius_m=self.trigger_rules.radius_m)):
                if session.window is not None:
                    session.window.pause_for_completion(lambda: self.approach.checkpoint(session))
                if self.clock()-session.last_confirmation >= self.limits.confirmation_interval_s:
                    session.confirmations += 1
                    session.last_confirmation = self.clock()
                self._event(trace, "confirming", state=VisualGoalState.VERIFYING,
                            count=session.confirmations, geometry=geometry)
                if session.confirmations >= self.limits.confirmation_count:
                    verdict = self.flight.verify(parameters, observation, goal, session, trace)
                    if verdict is None:
                        session.confirmations = 0
                        continue
                    if verdict:
                        self._event(trace, "completion_confirmed", remaining_points=len(session.pending_motion.segments))
                        session.pending_motion = MotionTrajectory(())
                        if session.window is not None:
                            session.window.complete()
                        session.completion_confirmed = True
                        return VisualGoalStatus.REACHED, "semantic completion confirmed after stopped bbox-center measurements"
                    session.trigger.checked_at(observation.vehicle_pose)
                else:
                    self._wait(self.limits.confirmation_interval_s, session)
                    continue
            session.confirmations = 0
            # Only post-execution approach rounds count toward distance progress.
            if session.translation_pending_progress:
                session.policy.record_progress(geometry.distance_m)
                session.translation_pending_progress = False
            if session.pending_motion.segments:
                self.flight.resume(observation, parameters, session, trace,
                    prefetch=lambda: self.approach.prefetch(observation, goal, parameters, session))
                continue
            context = VisualGoalPlanningContext(parameters.target_description, goal, geometry,
                VisualGoalState.APPROACHING, self.trigger_rules.radius_m, self.limits.distance_uncertainty_margin_m,
                self.limits.maximum_vertical_offset_m)
            self._event(trace, "state", state=VisualGoalState.APPROACHING)
            self.approach.run(observation, context, parameters, session, trace)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        callbacks = [self.model_calls.close]
        if self.window is not None:
            callbacks.append(self.window.close)
        if self.verification is not None:
            callbacks.append(self.verification.calls.close)
        close_resources(tuple(callbacks))

    def has_pending_calls(self):
        return (self.model_calls.has_pending_call or
                (self.window is not None and self.window.worker.has_pending_call) or
                (self.verification is not None and self.verification.calls.has_pending_call))

    def _search(self, observation, parameters, session, trace):
        if session.pending_motion.segments:
            # Search rotation would change a retained flight's anchor and evidence.
            raise VisualGoalFailure(VisualGoalStatus.TARGET_LOST, "target lost while paused; retained flight remains suspended")
        session.require_time(self.clock)
        heading = session.search.next_heading(observation.vehicle_pose, session.reference,
                                               parameters.target_description)
        relative = RelativeTrajectory((RelativePoseDelta(0, 0, 0, wrap_yaw_deg(heading-observation.vehicle_pose.yaw_deg)),))
        if self.execution.collision_state():
            raise ExecutionError("collision during enabled search check")
        if not session.policy.anchor_valid(observation, self.execution.current_pose(), self.clock()):
            return
        self.validator.validate_relative(relative)
        world = relative_to_world(relative, observation.vehicle_pose)
        self.flight.execute(self.motion.create_motion(world), session, trace, enable_completion_pause=False)
        self._event(trace, "search", heading=heading, moves=session.search.total_moves)

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
