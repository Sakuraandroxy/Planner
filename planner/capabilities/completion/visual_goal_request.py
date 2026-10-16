"""Explicit fresh-depth or stage-memory admission; neither proves semantic completion."""
import math

from planner.capabilities.approach import TargetApproach
from planner.domain.approach import ApproachSettings
from planner.domain.visual_goal import GoalVisibility
from planner.domain.visual_goal_completion import VisualGoalCompletionInput
from planner.errors import ProtocolError
from planner.domain.target_memory import TargetPositionMemory


def require_visual_completion_evidence(request: VisualGoalCompletionInput) -> None:
    message = "completion request requires stopped, source-labelled local completion conditions"
    if not isinstance(request, VisualGoalCompletionInput):
        raise ProtocolError(message)
    try:
        settings = ApproachSettings(request.rules.radius_m, request.uncertainty_margin_m)
        memory = request.target_memory
        if memory is not None and not isinstance(memory, TargetPositionMemory):
            raise ProtocolError(message)
        if memory is None:
            identity = request.goal.same_target and request.goal.status is GoalVisibility.VISIBLE
        else:
            pose = request.observation.vehicle_pose
            measured = math.dist((pose.x, pose.y, pose.z), memory.world_point)
            identity = (memory.stage_id == request.stage_id and memory.world_point == request.geometry.world_point
                and request.reference_rgb is not None
                and 0 <= memory.captured_monotonic <= request.observation.captured_monotonic
                and bool(request.observation.capture_id) and request.observation.timestamp_ns > 0
                and memory.timestamp_ns <= request.observation.timestamp_ns
                and math.isclose(measured, request.geometry.distance_m, abs_tol=1e-6, rel_tol=1e-9))
        valid = (request.observation.stopped and request.geometry.reliable and identity
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
