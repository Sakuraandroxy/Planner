import json
import math

from planner.domain import VisualGoalPlanningContext
from planner.errors import ProtocolError
from planner.algorithms.depth import FreeSpaceProfileSettings, measure_free_space
from planner.algorithms import point_in_horizontal_body
from planner.algorithms import relative_to_world

from .base import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    build_aerial_base_user_prompt,
    build_base_user_prompt,
)
from .free_navigation import format_free_space_profile
from .render import load, load_fragment, render

VISUAL_GOAL_SYSTEM_PROMPT = (
    BASE_TRAJECTORY_SYSTEM_PROMPT + "\n\n" + load("visual_goal_rules")
)


# UAV variant, ported from the reference Planner (/roxy/zhusai/Planner).
# The drone keeps vertical motion; the rover above keeps carla-vln's
# ground-vehicle rules. Both variants add the project's target-side dyaw
# constraint, which is not present in either reference.
AERIAL_VISUAL_GOAL_SYSTEM_PROMPT = (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT + "\n\n" + load("aerial_visual_goal_rules")
)


VISUAL_GOAL_VIEW_GUIDANCE = "\n" + load("visual_goal_view_guidance")


def _target_side_rule(target_vector) -> str:
    # The target side fixes the forbidden turn direction for this batch; a
    # zero turn stays available so straight motion is never blocked.
    forward, right, _ = target_vector
    if math.hypot(forward, right) <= 1e-6:
        return load_fragment("target_side_rule_undefined")
    if right > 0:
        return load_fragment("target_side_rule_right")
    if right < 0:
        return load_fragment("target_side_rule_left")
    return load_fragment("target_side_rule_ahead")


def _new_trajectory_reference(target_vector, has_pending):
    # Bearing is relative to the same yaw-only body frame as the NEW action anchor.
    forward, right, _ = target_vector
    bearing = (f"{math.degrees(math.atan2(right, forward)):+.2f} degrees"
               if math.hypot(forward, right) > 1e-6
               else "undefined (target horizontal distance is near zero; do not infer a turn)")
    start = load_fragment(
        "new_trajectory_start_pending" if has_pending else "new_trajectory_start_empty")
    return render(
        load_fragment("new_trajectory_reference"),
        start=start,
        target_vector_json=json.dumps(
            [round(value, 4) for value in target_vector], separators=(",", ":")),
        bearing=bearing,
        side_rule=_target_side_rule(target_vector),
    )


VISUAL_GOAL_OUTPUT_GUIDANCE = "\n" + load("visual_goal_output_guidance")
AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE = "\n" + load("aerial_visual_goal_output_guidance")


def visual_goal_system_prompt(*, aerial: bool = False) -> str:
    """Return the UAV or ground-vehicle visual-goal system prompt."""
    if aerial:
        return AERIAL_VISUAL_GOAL_SYSTEM_PROMPT
    return VISUAL_GOAL_SYSTEM_PROMPT


class VisualGoalTrajectoryPrompt:
    def __init__(self, *, aerial: bool = False,
                 free_space: FreeSpaceProfileSettings | None = None):
        self.aerial = aerial
        # Shared with free navigation: the same depth-derived obstacle fan
        # supplies the ground vehicle's metric clearance here too.
        self.free_space = free_space or FreeSpaceProfileSettings()

    def system_prompt(self):
        return visual_goal_system_prompt(aerial=self.aerial)

    def build_user_prompt(self, instruction, observation, max_points, *,
                          task_context=None, **execution_context):
        if not isinstance(task_context, VisualGoalPlanningContext):
            raise ProtocolError("visual goal prompt requires typed target context")
        base = (build_aerial_base_user_prompt if self.aerial else build_base_user_prompt)(
            instruction, observation, max_points, **execution_context)
        if not self.aerial:
            # Ground vehicle only: give the model measured metric obstacle
            # distances instead of leaving it to estimate scale from the image.
            base += "\n\n" + format_free_space_profile(
                measure_free_space(observation, self.free_space)
            )
        pending = execution_context.get("pending_trajectory")
        tail = (relative_to_world(pending, observation.pose).poses[-1]
                if pending is not None and pending.points else observation.pose)
        point = task_context.geometry.world_point
        if not task_context.geometry.reliable or point is None:
            raise ProtocolError("visual goal prompt requires an estimated target position")
        source_time = task_context.target_captured_monotonic
        payload = {
            "target_description": task_context.target_description,
            "distance_mode": "horizontal_to_nearest_known_surface",
            "target_surface_points_relative_to_observation_body_m": [point_in_horizontal_body(p,observation.pose)
                for p in (task_context.geometry.surface_points_world or (point,))],
            "target_surface_points_relative_to_pending_tail_body_m": [point_in_horizontal_body(p,tail)
                for p in (task_context.geometry.surface_points_world or (point,))],
            "keep_target_in_view": task_context.keep_target_in_view,
            "target_point_source": (task_context.geometry.position_source if task_context.goal_observation_is_current
                                    else "stage_bound_"+task_context.geometry.position_source),
            "target_observation_id": task_context.target_capture_id or (
                observation.capture_id if task_context.goal_observation_is_current else None),
            "planning_observation_id": observation.capture_id,
            "target_age_at_planning_observation_s": (None if source_time is None else
                round(observation.captured_monotonic-source_time, 4)),
            "coordinate_frame": "horizontal_body_forward_right_down",
            "target_relative_to_observation_body_m": point_in_horizontal_body(point, observation.pose),
            "target_relative_to_pending_tail_body_m": point_in_horizontal_body(point, tail),
            "new_actions_reference": "pending_tail" if pending is not None and pending.points else "observation",
            "pending_tail_is_predicted": bool(pending is not None and pending.points),
            "goal_observation_is_current": task_context.goal_observation_is_current,
            "arrival_radius_m": task_context.arrival_radius_m,
            "distance_uncertainty_margin_m": task_context.distance_uncertainty_margin_m,
            "maximum_vertical_offset_m": task_context.maximum_vertical_offset_m,
        }
        if payload["target_age_at_planning_observation_s"] is not None and payload["target_age_at_planning_observation_s"] < 0:
            raise ProtocolError("target estimate is newer than the planning observation")
        user = base + "\n" + render(
            load_fragment("visual_goal_context"),
            payload_json=json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        )
        if task_context.keep_target_in_view:
            user += "\n" + VISUAL_GOAL_VIEW_GUIDANCE.strip()
        has_pending = payload["pending_tail_is_predicted"]
        target_vector = payload["target_relative_to_pending_tail_body_m"] if has_pending else payload["target_relative_to_observation_body_m"]
        user += "\n\n" + _new_trajectory_reference(target_vector, has_pending)
        guidance = (
            AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE
            if self.aerial
            else VISUAL_GOAL_OUTPUT_GUIDANCE
        )
        return user + "\n\n" + guidance.strip()