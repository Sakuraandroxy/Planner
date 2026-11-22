"""Visual-goal completion: evidence admission, radius trigger, verification.

原 ``algorithms/completion`` 的五个模块合在此处，各自的作用见块首注释。
只做判定，不调用设备、队列或 HTTP 实现。
"""
import math
from dataclasses import replace

from planner.workflows.visual_goal_navigation.policy import TargetApproach
from planner.algorithms.target_coordinates import horizontal_distance
from planner.domain import ApproachSettings
from planner.domain import resolve_yaw_delta_deg, wrap_yaw_deg
from planner.domain import TargetPositionState
from planner.domain import WorldTrajectory
from planner.domain import GoalVisibility
from planner.domain import (
    VisualGoalCompletionInput,
    VisualGoalTargetSnapshot,
    VisualGoalTriggerRules,
)
from planner.domain import VisualGoalLimits
from planner.errors import ProtocolError
from planner.ports import TaskCompletionChecker
from planner.runtime.model_calls import ModelCallService


# Explicit fresh-depth or stage-bound admission; neither proves semantic completion.

def require_visual_completion_evidence(request: VisualGoalCompletionInput) -> None:
    message = "completion request requires stopped, source-labelled local completion conditions"
    if not isinstance(request, VisualGoalCompletionInput):
        raise ProtocolError(message)
    try:
        settings = ApproachSettings(request.rules.radius_m, request.uncertainty_margin_m)
        target_state = request.target_state
        if target_state is not None and not isinstance(target_state, TargetPositionState):
            raise ProtocolError(message)
        if target_state is None:
            identity = request.goal.same_target and request.goal.status is GoalVisibility.VISIBLE
            measured = horizontal_distance(request.geometry.world_point,request.observation.pose)
        else:
            pose = request.observation.pose
            measured = min(horizontal_distance(point,pose) for point in target_state.known_surfaces)
            identity = (target_state.stage_id == request.stage_id and request.geometry.world_point in target_state.known_surfaces
                and request.reference_rgb is not None
                and 0 <= target_state.captured_monotonic <= request.observation.captured_monotonic
                and bool(request.observation.capture_id) and request.observation.timestamp_ns > 0
                and target_state.timestamp_ns <= request.observation.timestamp_ns
                and math.isclose(measured, request.geometry.distance_m, abs_tol=1e-6, rel_tol=1e-9))
        valid = (request.observation.stopped and request.geometry.reliable and identity
                 and type(request.confirmation_count) is int
                 and type(request.required_confirmation_count) is int
                 and request.required_confirmation_count >= 2
                 and request.confirmation_count >= request.required_confirmation_count
                 and request.geometry.world_point is not None
                 and all(math.isfinite(v) for v in request.geometry.world_point)
                 and math.isclose(measured,request.geometry.distance_m,abs_tol=1e-6,rel_tol=1e-9)
                 and TargetApproach.distance_within_radius(measured, settings))
        if request.height_tolerance_m is not None:
            valid = (valid and not isinstance(request.height_tolerance_m, bool)
                     and isinstance(request.height_tolerance_m, (int, float))
                     and math.isfinite(request.height_tolerance_m) and request.height_tolerance_m > 0
                     and abs(request.observation.pose.z-request.initial_pose.z) <= request.height_tolerance_m)
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise ProtocolError(message)


# Task-specific stopped distance admission; visibility is judged by the model.

class VisualGoalCompletion:
    def __init__(self, limits: VisualGoalLimits, approach: TargetApproach | None = None):
        self.limits = limits
        self.approach = approach if approach is not None else TargetApproach()

    def candidate(self, parameters, observation, goal, geometry, initial_pose, *, radius_m: float, target_state=None):
        limits = self.limits
        if not observation.stopped or not geometry.reliable:
            return False
        if target_state is None:
            if goal.status is not GoalVisibility.VISIBLE or not goal.same_target:
                return False
        elif geometry.world_point not in target_state.known_surfaces:
            return False
        if (limits.maximum_vertical_offset_m is not None and
                abs(observation.pose.z-initial_pose.z) > limits.maximum_vertical_offset_m):
            return False
        return self.approach.distance_within_radius(horizontal_distance(geometry.world_point,observation.pose), limits.approach_settings(radius_m))


# Task-local radius pause policy; no device calls, network calls or queue access.

class VisualGoalRadiusTrigger:
    def __init__(self, rules: VisualGoalTriggerRules, maximum_age_s: float, clock):
        self.rules, self.maximum_age_s, self.clock = rules, maximum_age_s, clock
        self.snapshot: VisualGoalTargetSnapshot | None = None
        self.last_pause_pose = None
        self.last_reason: str | None = None
        self.armed = True

    def update(self, snapshot: VisualGoalTargetSnapshot):
        if self.snapshot is not None and self.snapshot.stage_id != snapshot.stage_id:
            raise ValueError("target snapshot belongs to another mission stage")
        self.snapshot = snapshot

    def checked_at(self, pose):
        """Latch the current check, including an initial observation already inside."""
        self.last_pause_pose = pose
        self.armed = False

    def should_pause(self, pose):
        snapshot = self.snapshot
        age = self.clock()-snapshot.captured_monotonic if snapshot else math.inf
        if snapshot is None or age < 0 or (not snapshot.stage_bound and age > self.maximum_age_s):
            self.last_reason = "target_reference_expired"
            return True
        distance = min(math.hypot(pose.x-point[0],pose.y-point[1]) for point in
                       (snapshot.surface_points_world or (snapshot.position_world,)))
        if distance > self.rules.effective_pause_radius_m:
            self.armed = True
            return False
        previous = self.last_pause_pose
        progressed = previous is None or (
            math.dist((pose.x, pose.y, pose.z), (previous.x, previous.y, previous.z)) >= self.rules.rearm_translation_m or
            abs(wrap_yaw_deg(pose.yaw_deg-previous.yaw_deg)) >= self.rules.rearm_yaw_deg)
        if self.armed or progressed:
            self.checked_at(pose)
            self.last_reason = "completion_radius"
            return True
        return False


# Re-anchor retained world goals without repeating already executed deltas.

def retained_world_trajectory(pending, actual):
    if not pending.segments:
        raise ValueError("no retained trajectory")
    first = pending.segments[0]
    # Remaining signed yaw already excludes the rotation measured before pause.
    remaining_yaw = first.yaw_delta_deg
    if remaining_yaw is None:
        remaining_yaw = resolve_yaw_delta_deg(first.start.yaw_deg, first.end.yaw_deg)
    deltas = (remaining_yaw+resolve_yaw_delta_deg(actual.yaw_deg, first.start.yaw_deg), *(
        segment.yaw_delta_deg if segment.yaw_delta_deg is not None else
        resolve_yaw_delta_deg(segment.start.yaw_deg, segment.end.yaw_deg) for segment in pending.segments[1:]))
    return WorldTrajectory(actual, tuple(segment.end for segment in pending.segments), tuple(deltas))


# Semantic check over frozen local evidence; no simulator or HTTP implementation.

class VisualGoalVerification:
    def __init__(self, checker: TaskCompletionChecker[VisualGoalCompletionInput], clock):
        self.checker = checker
        self.calls = ModelCallService(clock)

    def check(self, request, budget, timeout_s, retry_count, checkpoint, clock):
        require_visual_completion_evidence(request)
        # The worker sees copies, never a mutable control session or device.
        depth = request.observation.depth_m.copy()
        depth.setflags(write=False)
        depth_image = request.observation.depth_image
        if depth_image is not None:
            depth_image = depth_image.copy()
        observed = replace(request.observation, rgb=request.observation.rgb.copy(),
                           depth_image=depth_image, depth_m=depth)
        frozen = replace(request, observation=observed,
            reference_rgb=request.reference_rgb.copy() if request.reference_rgb is not None else None)
        for attempt in range(retry_count+1):
            timeout = min(timeout_s, budget.remaining(clock()))
            try:
                value = self.calls.call(lambda timeout=timeout: self.checker.check(frozen, timeout_s=timeout), timeout, checkpoint)
                if type(value) is not bool:
                    raise ProtocolError("completion checker must return a boolean")
                return value
            except ProtocolError:
                if attempt >= retry_count:
                    raise
        raise AssertionError("unreachable completion retry state")
