"""Explicit fresh-depth or stage-bound admission; neither proves semantic completion."""
import math

from planner.capabilities.approach import TargetApproach
from planner.domain.approach import ApproachSettings
from planner.domain.visual_goal import GoalVisibility
from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from planner.errors import ProtocolError
from planner.domain.target_state import TargetPositionState
from planner.services.target_coordinates import horizontal_distance


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