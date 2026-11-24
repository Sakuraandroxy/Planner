"""Bare visual-goal navigation: instruction-only prompt, engine path following.

One round captures a frame, binds the target once, asks the model once, converts
the returned offsets to world points without any waypoint validation, hands the
whole batch to the engine, publishes it, and stops when the measured distance to
the bound target enters the task radius.

Deliberately absent: pending-queue planning, window smoothing, collision checks,
waypoint validation and semantic completion confirmation.
"""
import logging
import math
import time
from functools import partial

from planner.capabilities.targeting.association import (
    accept_position,
    bound_target_geometry,
)
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.domain.execution import ExecutionBudget, StopResult
from planner.domain.mission import MissionStage
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.result import (
    CompletionBasis,
    ResultMeasurement,
    StageResult,
    StageSummary,
)
from planner.domain.sliding_window import QueuedWaypoint
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.trajectory_telemetry import MODE_STOPPED_BATCH
from planner.domain.visual_goal import (
    GoalVisibility,
    VisualGoalNavigationParameters,
    VisualGoalNavigationResult,
    VisualGoalStatus,
)
from planner.domain.visual_goal_completion import VisualGoalTriggerRules
from planner.domain.visual_goal_limits import BareNavigationSettings, VisualGoalLimits
from planner.errors import (
    BudgetExceeded,
    ExecutionError,
    ModelCallTimeout,
    ObservationError,
    ProtocolError,
)
from planner.ports.goal_observer import GoalObserver
from planner.ports.observation_source import ObservationSource
from planner.ports.path_vehicle import PathVehicle
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.execution import ExecutionService
from planner.services.model_calls import ModelCallService
from planner.services.trajectory_transform import offsets_to_world

from .session import VisualGoalSession, model_call
from .termination import VisualGoalExitPolicy, VisualGoalFailure

logger = logging.getLogger(__name__)

# Baseline semantics: the engine receives the whole batch but only the first
# five waypoints are executed. The speed is the bare profile's own value.
_BATCH_POINTS = 5


class BareVisualGoalNavigation:
    def __init__(self, observation_source: ObservationSource,
                 goal_observer: GoalObserver,
                 trajectory_planner: TrajectoryPlanner,
                 path_vehicle: PathVehicle,
                 geometry: DepthGeometry,
                 execution: ExecutionService, *,
                 trigger_rules: VisualGoalTriggerRules = VisualGoalTriggerRules(),
                 limits: VisualGoalLimits = VisualGoalLimits(),
                 bare: BareNavigationSettings = BareNavigationSettings(),
                 clock=time.monotonic, telemetry=None, vertical_motion: bool = True):
        self.source = observation_source
        self.goal_observer = goal_observer
        self.planner = trajectory_planner
        self.path_vehicle = path_vehicle
        self.geometry = geometry
        self.execution = execution
        self.trigger_rules = trigger_rules
        self.limits = limits
        self.bare = bare
        self.clock = clock
        self.telemetry = telemetry
        self.vertical_motion = bool(vertical_motion)
        self.model_calls = ModelCallService(clock)
        self._closed = False
        self._last_pose: WorldPose | None = None

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("bare visual goal workflow is closed")
        # A request the previous stage left quarantined has to fail here, not on
        # the first round: the model caller is single-flight.
        self.model_calls.require_available()
        if not callable(getattr(self.path_vehicle, "follow_path", None)):
            raise ExecutionError("bare navigation requires a path-following vehicle")

    def run(self, stage: MissionStage) -> StageResult[object]:
        started = self.clock()
        session = VisualGoalSession(
            ExecutionBudget(started + self.limits.maximum_task_duration_s),
            VisualGoalExitPolicy(self.limits), started=started)
        session.stage_id = stage.stage_id
        # A stop request reaches this profile through the same two checkpoints the
        # staged pipeline binds: vehicle hold/observation, plus every explicit
        # ``checkpoint()`` call below and inside each model call.
        session.checkpoint = self.execution.checkpoint
        session.model_calls = self.model_calls
        self._last_pose = None
        try:
            self.prepare()
            parameters = stage.parameters
            if not isinstance(parameters, VisualGoalNavigationParameters):
                raise VisualGoalFailure(VisualGoalStatus.BLOCKED, "invalid visual goal parameters")
            self.limits.validate_arrival_radius(self.trigger_rules.radius_m)
            status, reason = self._navigate(parameters, session)
        except (Exception, KeyboardInterrupt) as exc:
            status = _failure_status(exc)
            reason = str(exc) or status.value
            if session.target_state is None:
                reason = f"{reason}; no target was ever bound"
        distance = None
        if session.target_state is not None and self._last_pose is not None:
            distance = bound_target_geometry(session.target_state, self._last_pose).distance_m
        elapsed = self.clock() - started
        success = status is VisualGoalStatus.REACHED
        logger.info("[BareVisualGoal %s] %s: %s distance=%s rounds=%d elapsed=%.1fs",
                    stage.stage_id, status.value, reason, distance,
                    session.policy.planning_calls, elapsed)
        result = VisualGoalNavigationResult(
            status, reason, distance, session.policy.planning_calls, elapsed,
            session.path_length_m, 0,
            StopResult(False, ("the bare profile does not verify a stopped vehicle",)),
            evidence_source="instruction-only prompt with a local arrival radius",
            path_length_source="sum_of_planned_leg_chords_not_measured_driving",
            completion_confirmed=success,
        )
        summary = StageSummary(
            status.value, reason,
            CompletionBasis.LOCAL_MEASUREMENT if success else CompletionBasis.NOT_VERIFIED,
            success,
            measurements=(
                ResultMeasurement("distance", distance, "m"),
                ResultMeasurement("elapsed", elapsed, "s"),
                ResultMeasurement("planning_calls", session.policy.planning_calls),
            ),
            final_pose=self._last_pose,
        )
        return StageResult(stage.stage_id, success, result,
                           None if success else f"{status.value}: {reason}", summary)

    def close(self) -> None:
        self._closed = True
        self.model_calls.close()

    def _navigate(self, parameters, session):
        # ``previous`` is the last batch the engine accepted, kept as body-frame
        # offsets. It carries "target lost keeps the previous direction": a round
        # whose detection does not see the target re-sends that batch from the
        # current pose and does not ask the model. Detection runs every round for
        # exactly this reason -- visibility is only observable from the detector's
        # answer, not from the bound position. Rounds without an earlier batch
        # still ask the model, and the time budget ends a run whose target never
        # comes back.
        previous: RelativeTrajectory | None = None
        while True:
            self.execution.checkpoint()
            session.require_time(self.clock)
            observation = self.source.capture(budget=session.budget)
            session.require_time(self.clock)
            if not session.fresh(observation):
                session.policy.unreliable_observation()
                continue
            self._last_pose = observation.pose
            if session.initial is None:
                session.initial = observation.pose
            finished = self._finish_if_close(parameters, session, observation.pose)
            if finished is not None:
                return finished
            goal = self._observe(observation, parameters, session)
            if session.target_state is None:
                self._bind_target(observation, goal, session)
                finished = self._finish_if_close(parameters, session, observation.pose)
                if finished is not None:
                    return finished
            visible = (goal is not None and goal.status is GoalVisibility.VISIBLE
                       and goal.same_target)
            if not visible and previous is not None:
                trajectory = previous
            else:
                trajectory = self._plan(observation, parameters, session)
            self.execution.checkpoint()
            batch = self._world_batch(trajectory, observation.pose)
            session.path_length_m += _leg_length_m(observation.pose, batch)
            self._follow(batch, session)
            previous = trajectory
            self._publish(parameters, session, observation.pose, batch)

    def _finish_if_close(self, parameters, session, pose):
        """Return the success pair once the bound target is inside the radius."""
        if session.target_state is None:
            return None
        distance = bound_target_geometry(session.target_state, pose).distance_m
        if distance > self.trigger_rules.radius_m:
            return None
        self._publish(parameters, session, pose, ())
        return (VisualGoalStatus.REACHED,
                f"measured distance {distance:.2f} m within radius "
                f"{self.trigger_rules.radius_m:.2f} m")

    def _observe(self, observation, parameters, session):
        """Run this round's detection; the loop reads visibility from its answer."""
        operation = partial(
            self.goal_observer.observe, observation, parameters.target_description, None,
            budget=session.budget, initial_pose=session.initial,
            instruction=parameters.instruction)
        try:
            return model_call(operation, session, self.limits, clock=self.clock)
        except (VisualGoalFailure, ModelCallTimeout):
            if session.target_state is None:
                raise
            # A detector outage after binding is a lost target for this round:
            # keep flying the previous batch instead of ending the run.
            logger.info("[BareVisualGoal %s] detection failed; reusing the previous batch",
                        session.stage_id)
            return None

    def _bind_target(self, observation, goal, session) -> None:
        """Bind the first accepted detection; a bound position is never replaced."""
        if goal is None:
            return
        geometry = self.geometry.estimate(observation, goal)
        target_state, reason = accept_position(
            session.target_state, session.stage_id, observation, goal, geometry,
            self.limits.target_position_update_radius_m)
        if target_state is None or reason is not None:
            logger.info("[BareVisualGoal %s] target not bound this round: %s",
                        session.stage_id, reason)
            return
        session.target_state = target_state

    def _plan(self, observation, parameters, session):
        # One planning round per model request, retries included: the round index
        # tells the model how many requests the stage has already spent.
        session.policy.planning_call()
        operation = partial(
            self.planner.plan, observation, parameters.instruction,
            round_index=session.policy.planning_calls,
            initial_pose=session.initial, budget=session.budget)
        return model_call(operation, session, self.limits, clock=self.clock)

    def _world_batch(self, trajectory, pose):
        points = tuple(trajectory.points[:_BATCH_POINTS])
        if not points:
            raise VisualGoalFailure(VisualGoalStatus.MODEL_ERROR,
                                    "trajectory parse returned no waypoints")
        return tuple(offsets_to_world(RelativeTrajectory(points), pose).poses)

    def _follow(self, batch, session) -> None:
        self.execution.checkpoint()
        session.require_time(self.clock)
        try:
            self.path_vehicle.follow_path(
                batch, self.bare.speed_mps, timeout_s=self.bare.path_timeout_s)
        except Exception as exc:
            raise ExecutionError(
                f"engine rejected or timed out on path following: {exc}") from exc
        session.require_time(self.clock)
        self.execution.checkpoint()

    def _publish(self, parameters, session, pose, batch) -> None:
        if self.telemetry is None:
            return
        items = []
        start = pose
        for index, end in enumerate(batch, start=1):
            items.append(QueuedWaypoint(
                waypoint_id=index, start=start, end=end,
                yaw_delta_deg=wrap_yaw_deg(end.yaw_deg - start.yaw_deg)))
            start = end
        self.telemetry.publish_queue(
            instruction=parameters.instruction,
            stage_id=session.stage_id,
            mode=MODE_STOPPED_BATCH,
            capacity=len(items),
            pose=pose,
            items=tuple(items),
            planning_calls=session.policy.planning_calls,
            path_length_m=session.path_length_m,
            vertical_motion=self.vertical_motion,
        )


def _leg_length_m(start: WorldPose, batch) -> float:
    length = 0.0
    previous = start
    for pose in batch:
        length += math.dist((previous.x, previous.y, previous.z),
                            (pose.x, pose.y, pose.z))
        previous = pose
    return length


def _failure_status(exc) -> VisualGoalStatus:
    if isinstance(exc, VisualGoalFailure):
        return exc.status
    if isinstance(exc, (BudgetExceeded, ModelCallTimeout)):
        return VisualGoalStatus.TIMEOUT
    if isinstance(exc, ObservationError):
        return VisualGoalStatus.OBSERVATION_ERROR
    if isinstance(exc, ProtocolError):
        return VisualGoalStatus.MODEL_ERROR
    if isinstance(exc, KeyboardInterrupt):
        return VisualGoalStatus.CANCELLED
    if isinstance(exc, ExecutionError):
        return VisualGoalStatus.EXECUTION_ERROR
    return VisualGoalStatus.BLOCKED
