"""Admission checks for visual completion evidence, including radius and fresh depth."""
import math

from planner.capabilities.approach import TargetApproach
from planner.domain.approach import ApproachSettings
from planner.domain.visual_goal import GoalVisibility
from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from planner.errors import ProtocolError


def require_visual_completion_evidence(request: VisualGoalCompletionInput) -> None:
    message = "completion request requires verified fresh local completion conditions"
    if not isinstance(request, VisualGoalCompletionInput):
        raise ProtocolError(message)
    try:
        settings = ApproachSettings(request.rules.radius_m, request.uncertainty_margin_m)
        valid = (request.observation.stopped and request.geometry.reliable and request.goal.same_target
                 and request.goal.status is GoalVisibility.VISIBLE
                 and type(request.confirmation_count) is int
                 and type(request.required_confirmation_count) is int
                 and request.required_confirmation_count >= 2
                 and request.confirmation_count >= request.required_confirmation_count
                 and request.geometry.world_point is not None
                 and all(math.isfinite(v) for v in request.geometry.world_point)
                 and TargetApproach.distance_within_radius(request.geometry.distance_m, settings))
        if request.height_tolerance_m is not None:
            valid = (valid and not isinstance(request.height_tolerance_m, bool)
                     and isinstance(request.height_tolerance_m, (int, float))
                     and math.isfinite(request.height_tolerance_m) and request.height_tolerance_m > 0
                     and abs(request.observation.vehicle_pose.z-request.initial_pose.z) <= request.height_tolerance_m)
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise ProtocolError(message)
