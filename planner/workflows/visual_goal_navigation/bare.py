"""Bare visual-goal navigation: instruction-only prompt, engine path following.

One round captures a frame, binds the target once, asks the model once, converts
the returned relative waypoints to world points without any waypoint validation,
hands the whole batch to the engine, publishes it, and stops when the measured
distance to the bound target enters the task radius.

Deliberately absent: pending-queue planning, window smoothing, collision checks,
waypoint validation and semantic completion confirmation.
"""
import logging
import math
import time

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
from planner.ports.trajectory_planner import TrajectoryPlanner
from planner.services.trajectory_transform import relative_to_world

from .session import VisualGoalSession
from .termination import SearchPolicy, VisualGoalExitPolicy, VisualGoalFailure

logger = logging.getLogger(__name__)

# Baseline semantics: the engine receives the whole batch but only the first
# five waypoints are executed. The speed is the bare profile's own value.
_BATCH_POINTS = 5


class BareVisualGoalNavigation:
    def __init__(self, observation_source: ObservationSource,
                 goal_observer: GoalObserver,
                 trajectory_planner: TrajectoryPlanner,
                 path_vehicle,
                 geometry: DepthGeometry, *,
                 trigger_rules: VisualGoalTriggerRules = VisualGoalTriggerRules(),
                 limits: VisualGoalLimits = VisualGoalLimits(),
                 bare: BareNavigationSettings = BareNavigationSettings(),
                 clock=time.monotonic, telemetry=None, vertical_motion: bool = True):
        self.source = observation_source
        self.goal_observer = goal_observer
        self.planner = trajectory_planner
        self.path_vehicle = path_vehicle
        self.geometry = geometry
        self.trigger_rules = trigger_rules
        self.limits = limits
        self.bare = bare
        self.clock = clock
        self.telemetry = telemetry
        self.vertical_motion = bool(vertical_motion)
        self._closed = False
        self._last_pose: WorldPose | None = None

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("bare visual goal workflow is closed")
        if not callable(getattr(self.path_vehicle, "follow_path", None)):
            raise ExecutionError("bare navigation requires a path-following vehicle")

    def run(self, stage: MissionStage) -> StageResult[object]:
        started = self.clock()
        session = VisualGoalSession(
            ExecutionBudget(started + self.limits.maximum_task_duration_s),
            VisualGoalExitPolicy(self.limits), SearchPolicy(self.limits), started)
        session.stage_id = stage.stage_id
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
            if status is VisualGoalStatus.TIMEOUT and session.target_state is None:
                reason = f"no target was ever bound; {reason}"
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

    def _navigate(self, parameters, session):
        while True:
            session.require_time(self.clock)
            session.policy.planning_call()
            observation = self.source.capture(budget=session.budget)
            session.require_time(self.clock)
            self._last_pose = observation.pose
            if session.initial is None:
                session.initial = observation.pose
            if session.target_state is None:
                self._bind_target(observation, parameters, session)
            else:
                distance = bound_target_geometry(session.target_state, observation.pose).distance_m
                if distance <= self.trigger_rules.radius_m:
                    self._publish(parameters, session, observation.pose, ())
                    return (VisualGoalStatus.REACHED,
                            f"measured distance {distance:.2f} m within radius "
                            f"{self.trigger_rules.radius_m:.2f} m")
            trajectory = self._plan(observation, parameters, session)
            batch = self._world_batch(trajectory, observation.pose)
            session.path_length_m += _leg_length_m(observation.pose, batch)
            self._publish(parameters, session, observation.pose, batch)
            self._follow(batch)

    def _bind_target(self, observation, parameters, session) -> None:
        """Bind the first accepted detection; a bound target is never replaced."""
        goal = self.goal_observer.observe(
            observation, parameters.target_description, None,
            budget=session.budget, initial_pose=session.initial,
            instruction=parameters.instruction)
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
        try:
            return self.planner.plan(
                observation, parameters.instruction,
                round_index=session.policy.planning_calls,
                initial_pose=session.initial, budget=session.budget)
        except ProtocolError as exc:
            raise VisualGoalFailure(VisualGoalStatus.MODEL_ERROR,
                                    f"trajectory parse failed: {exc}") from exc

    def _world_batch(self, trajectory, pose):
        points = tuple(trajectory.points[:_BATCH_POINTS])
        if not points:
            raise VisualGoalFailure(VisualGoalStatus.MODEL_ERROR,
                                    "trajectory parse returned no waypoints")
        return tuple(relative_to_world(RelativeTrajectory(points), pose).poses)

    def _follow(self, batch) -> None:
        try:
            self.path_vehicle.follow_path(
                batch, self.bare.speed_mps, timeout_s=self.bare.path_timeout_s)
        except Exception as exc:
            raise ExecutionError(
                f"engine rejected or timed out on path following: {exc}") from exc

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
        length += math.hypot(pose.x - previous.x, pose.y - previous.y)
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
