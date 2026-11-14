import json
import math
from planner.domain.visual_goal import VisualGoalPlanningContext
from planner.errors import ProtocolError
from planner.services.target_coordinates import point_in_horizontal_body
from planner.services.trajectory_transform import relative_to_world

from .base import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    build_aerial_base_user_prompt,
    build_base_user_prompt,
)


VISUAL_GOAL_SYSTEM_PROMPT = BASE_TRAJECTORY_SYSTEM_PROMPT + """

Static visual goal approach rules:
The objective is to enter arrival_radius_m around the target's estimated position and stop.
Any position inside that radius is acceptable; do not aim for its exact boundary.
Use distance_uncertainty_margin_m for the arrival estimate. Preserve the selected object's identity.
The target is a locally estimated surface sample, not the object's 3D center or ground truth.
All initially accepted valid surface points remain bound to the selected object.
Use the supplied surface point vectors as a set. The single target vector selects the nearest
known surface horizontally at the observation pose; another supplied surface can be nearer
after pending motion. Re-evaluate this geometry at the NEW action reference.
Several samples do not describe the full object boundary or unseen surfaces.
Its first accepted position anchors the static target. Missing or rejected detection keeps
the last accepted position; its source and age do not prove current visibility.
Use the supplied target vectors [forward,right,down] in meters in the horizontal body frame.
target_relative_to_observation_body_m describes the image-time pose;
target_relative_to_pending_tail_body_m describes the predicted final pending pose.
With pending, NEW actions start at that predicted pose. Without pending, they start at the image pose.
The NEW trajectory planning reference below selects the starting target vector and bearing.
Use these estimates rather than re-estimating target distance or transforming world coordinates.
RGB, depth and camera geometry still inform scene/path choices and actual framing.
Initial directional descriptions identify the target at stage start; choose the current direction
from the planning reference rather than preserving its initial bearing.
If keep_target_in_view=true, coordinate translation and yaw to keep the bound target in front view.
When the planning reference places the target on the left side, never output a positive
dyaw_deg; when it places the target on the right side, never output a negative dyaw_deg.
Straight motion with dyaw_deg = 0 and turns toward the target remain allowed.
Plan useful progress toward the target and reduce motion near the arrival radius.
Arrival distance is horizontal distance to the nearest known target surface. Vertical separation
does not enter that radius. Surface coordinates still inform scene geometry.
This ground vehicle cannot change height; vertical motion is unavailable and dz must remain 0.
maximum_vertical_offset_m is therefore not a motion-planning constraint.
Choose useful evidence-supported motion; forward motion is not mandatory.
Do not plan blind lateral or backward motion into unobserved space.
The executor follows valid trajectory points in order, with radius pauses for completion checks.
Use the execution history and pending to determine what was actually driven.
Previous executed command prefix is not an exact measured driving path; use actual pose updates.
Do not output completion status: the independent completion checker evaluates it after local measurements.

Mandatory trajectory output format:
Return exactly one JSON array containing exactly 5 waypoint arrays.
Each waypoint array must contain exactly 4 finite JSON numbers:
[dx, dy, dz, dyaw_deg], with dz = 0 for this ground vehicle.

Enclose all 5 waypoint arrays in ONE outer array.
Return the entire result on one line, starting with [[ and ending with ]].
Do not return five separate arrays without the outer brackets.

Output only the final numerical trajectory.
Do not output scene descriptions, calculations, reasoning, action descriptions,
introductions, Markdown, code fences, comments, field names, or completion status.
Do not output placeholders, ellipses, strings, booleans, null, NaN, or Infinity.
"""


# UAV variant, ported from the reference Planner (/roxy/zhusai/Planner).
# The drone keeps vertical motion; the rover above keeps carla-vln's
# ground-vehicle rules. Both variants add the project's target-side dyaw
# constraint, which is not present in either reference.
AERIAL_VISUAL_GOAL_SYSTEM_PROMPT = AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT + """

Static visual goal approach rules:
The objective is to enter arrival_radius_m around the target's estimated position and hover.
Any position inside that radius is acceptable; do not aim for its exact boundary.
Use distance_uncertainty_margin_m for the arrival estimate. Preserve the selected object's identity.
The target is a locally estimated surface sample, not the object's 3D center or ground truth.
All initially accepted valid surface points remain bound to the selected object.
Use the supplied surface point vectors as a set. The single target vector selects the nearest
known surface horizontally at the observation pose; another supplied surface can be nearer
after pending motion. Re-evaluate this geometry at the NEW action reference.
Several samples do not describe the full object boundary or unseen surfaces.
Its first accepted position anchors the static target. Missing or rejected detection keeps
the last accepted position; its source and age do not prove current visibility.
Use the supplied target vectors [forward,right,NED down] in meters in the horizontal body frame.
target_relative_to_observation_body_m describes the image-time pose;
target_relative_to_pending_tail_body_m describes the predicted final pending pose.
With pending, NEW actions start at that predicted pose. Without pending, they start at the image pose.
The NEW trajectory planning reference below selects the starting target vector and bearing.
Use these estimates rather than re-estimating target distance or transforming world coordinates.
RGB, depth and camera geometry still inform scene/path choices and actual framing.
Initial directional descriptions identify the target at stage start; choose the current direction
from the planning reference rather than preserving its initial bearing.
If keep_target_in_view=true, coordinate translation and yaw to keep the bound target in front view.
When the planning reference places the target on the left side, never output a positive
dyaw_deg; when it places the target on the right side, never output a negative dyaw_deg.
Straight motion with dyaw_deg = 0 and turns toward the target remain allowed.
Plan useful progress toward the target and reduce motion near the arrival radius.
Arrival distance is horizontal distance to the nearest known target surface. Vertical separation
does not enter that radius. Surface coordinates still inform altitude and scene geometry.
Upward and downward motion are allowed. If maximum_vertical_offset_m is not null, stay
within that configured offset from stage-start height; null adds no such height restriction.
Choose useful evidence-supported motion; forward motion is not mandatory.
Do not plan blind lateral or backward flight into unobserved space.
The executor follows valid trajectory points in order, with radius pauses for completion checks.
Use the execution history and pending to determine what was actually flown.
Previous executed command prefix is not an exact measured flight path; use actual pose updates.
Do not output completion status: the independent completion checker evaluates it after local measurements.

Mandatory trajectory output format:
Return exactly one JSON array containing exactly 5 waypoint arrays.
Each waypoint array must contain exactly 4 finite JSON numbers:
[dx, dy, dz, dyaw_deg].

Enclose all 5 waypoint arrays in ONE outer array.
Return the entire result on one line, starting with [[ and ending with ]].
Do not return five separate arrays without the outer brackets.

Output only the final numerical trajectory.
Do not output scene descriptions, calculations, reasoning, action descriptions,
introductions, Markdown, code fences, comments, field names, or completion status.
Do not output placeholders, ellipses, strings, booleans, null, NaN, or Infinity.
"""


VISUAL_GOAL_VIEW_GUIDANCE = """
Visual goal viewing guidance:
While approaching, actively adjust dyaw_deg to keep the bound target in the front camera view,
preferably near the horizontal image center.
For a large heading correction, reduce translation or use a pure-yaw action as appropriate.
Little or no turn is appropriate when the target remains aligned after translation.
Yaw controls horizontal heading only; it cannot correct vertical framing or prove visibility.
"""


def _target_side_rule(target_vector) -> str:
    # The target side fixes the forbidden turn direction for this batch; a
    # zero turn stays available so straight motion is never blocked.
    forward, right, _ = target_vector
    if math.hypot(forward, right) <= 1e-6:
        return ("The target horizontal direction is undefined here; no dyaw_deg side "
                "restriction applies.")
    if right > 0:
        return ("The target is currently on the RIGHT side. Negative dyaw_deg (turning left, "
                "away from the target) is forbidden in this NEW trajectory; dyaw_deg = 0 "
                "is allowed.")
    if right < 0:
        return ("The target is currently on the LEFT side. Positive dyaw_deg (turning right, "
                "away from the target) is forbidden in this NEW trajectory; dyaw_deg = 0 "
                "is allowed.")
    return "The target is directly ahead or behind; no dyaw_deg side restriction applies."


def _new_trajectory_reference(target_vector, has_pending):
    # Bearing is relative to the same yaw-only body frame as the NEW action anchor.
    forward, right, _ = target_vector
    bearing = (f"{math.degrees(math.atan2(right, forward)):+.2f} degrees"
               if math.hypot(forward, right) > 1e-6
               else "undefined (target horizontal distance is near zero; do not infer a turn)")
    start = (
        "The existing pending actions are already scheduled and will execute first.\n"
        "Start your NEW trajectory AFTER all pending actions, at their predicted final pose.\n"
        "Do not return or replace the pending actions."
        if has_pending else
        "Pending is empty. Start your NEW trajectory at the current observation pose."
    )
    return (
        "NEW trajectory planning reference:\n" + start + "\n"
        "At this NEW trajectory start:\n"
        "- Target relative position [forward,right,down], meters: "
        + json.dumps([round(v, 4) for v in target_vector], separators=(",", ":")) + "\n"
        "- Target horizontal bearing: " + bearing + ".\n"
        "Positive bearing means right; negative bearing means left.\n"
        + _target_side_rule(target_vector) + "\n"
        "This bearing describes the starting geometry, not a required dyaw_deg command.\n"
        "The original directional description, such as 'front-left', identifies the object at mission start.\n"
        "It does not prescribe the direction of subsequent actions.\n"
        "dyaw_deg is an incremental turn relative to the preceding pose. Turns accumulate across waypoints.\n"
        "For example, five actions with -8 degrees each produce a total turn of -40 degrees.\n"
        "Re-evaluate the target direction after each predicted translation and turn.\n"
        "Do not repeat a turn merely because the previous action turned that way."
    )


VISUAL_GOAL_OUTPUT_GUIDANCE = """
Final response:
Output only one complete JSON array of exactly 5 NEW waypoints.
Each waypoint contains exactly 4 finite numbers: [dx, dy, dz, dyaw_deg],
with dz = 0 because this is a ground vehicle.
Start with [[ and end with ]]. Include the outer brackets.
Do not include pending waypoints, explanations, headings, Markdown, or code fences.

Correct output example:
The following example demonstrates the output format only.
Choose the actual waypoint values according to the current task, images,
target position and pending actions; do not treat the example as the requested trajectory.

[[0.5,0.1,0.0,5.0],[0.6,0.1,0.0,4.0],[0.5,0.0,0.0,2.0],[0.3,-0.1,0.0,-1.0],[0.1,0.0,0.0,0.0]]
"""


AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE = """
Final response:
Output only one complete JSON array of exactly 5 NEW waypoints.
Each waypoint contains exactly 4 finite numbers: [dx, dy, dz, dyaw_deg].
Start with [[ and end with ]]. Include the outer brackets.
Do not include pending waypoints, explanations, headings, Markdown, or code fences.

Correct output example:
The following example demonstrates the output format only.
Choose the actual waypoint values according to the current task, images,
target position and pending actions; do not treat the example as the requested trajectory.

[[0.5,0.1,-0.2,5.0],[0.6,0.1,-0.1,4.0],[0.5,0.0,0.0,2.0],[0.3,-0.1,0.0,-1.0],[0.1,0.0,0.0,0.0]]
"""


def visual_goal_system_prompt(*, aerial: bool = False) -> str:
    """Return the UAV or ground-vehicle visual-goal system prompt."""
    if aerial:
        return AERIAL_VISUAL_GOAL_SYSTEM_PROMPT
    return VISUAL_GOAL_SYSTEM_PROMPT


class VisualGoalTrajectoryPrompt:
    def __init__(self, *, aerial: bool = False):
        self.aerial = aerial

    def system_prompt(self):
        return visual_goal_system_prompt(aerial=self.aerial)

    def build_user_prompt(self, instruction, observation, max_points, *,
                          task_context=None, **execution_context):
        if not isinstance(task_context, VisualGoalPlanningContext):
            raise ProtocolError("visual goal prompt requires typed target context")
        base = (build_aerial_base_user_prompt if self.aerial else build_base_user_prompt)(
            instruction, observation, max_points, **execution_context)
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
        user = base + "\nVisual goal context (estimated evidence, not target ground truth):\n" + json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"),
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