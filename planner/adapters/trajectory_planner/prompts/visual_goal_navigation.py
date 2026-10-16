import json
from planner.domain.visual_goal import VisualGoalPlanningContext
from planner.errors import ProtocolError
from planner.services.target_coordinates import point_in_horizontal_body
from planner.services.trajectory_transform import relative_to_world

from .base import BASE_TRAJECTORY_SYSTEM_PROMPT, build_base_user_prompt


VISUAL_GOAL_SYSTEM_PROMPT = BASE_TRAJECTORY_SYSTEM_PROMPT + """

Static visual goal approach rules:
The objective is to enter arrival_radius_m around the target's estimated position and hover.
Any position inside that radius is acceptable; do not aim for its exact boundary.
Use distance_uncertainty_margin_m for the arrival estimate. Preserve the selected object's identity.
The target position is estimated locally from a detected bbox-center depth sample.
It is a surface estimate, not the object's true 3D center or a persistent landmark.
The first accepted position anchors this static target for the mission stage.
When later detection is missing or rejected, continue toward the last accepted position.
Its source capture and age stay unchanged; do not infer current visibility from memory.
Use the supplied horizontal-body relative target positions; do not re-estimate target distance
or transform world target coordinates yourself. RGB and depth still inform scene/path choices.
target_relative_to_observation_body_m refers to the image-time vehicle pose.
target_relative_to_pending_tail_body_m refers to the PREDICTED final pending pose.
With pending, NEW actions start at that predicted pose. Without pending, they start at the image pose.
Both target vectors are [forward,right,NED down] in meters, independent of camera pitch.
If keep_target_in_view=true, coordinate position and yaw to keep the bound target in the front view.
Initial directional descriptions identify the target at stage start; update its current
bearing from the current observation and geometry, rather than preserving its initial bearing.
Use measured target geometry as evidence, not as guaranteed ground truth. Plan useful
progress toward the target and reduce motion near the arrival radius.
Upward and downward motion are allowed. If maximum_vertical_offset_m is not null, stay
within that configured offset from stage-start height; null adds no such height restriction.
Choose useful evidence-supported motion; forward motion is not mandatory.
Do not plan blind lateral or backward flight into unobserved space.
The executor follows valid trajectory points in order, with radius pauses for completion checks.
Use the execution history and pending to determine what was actually flown.
Previous executed command prefix is not an exact measured flight path; use actual pose updates.
Do not output completion status: the independent completion checker evaluates it after local measurements.
The target may come from an older observation; its source and age are supplied explicitly.
An older target estimate is not proof of current image visibility. Completion is judged separately.

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
For the first NEW action, use target_relative_to_pending_tail_body_m when pending exists;
otherwise use target_relative_to_observation_body_m.
For each NEW action, express its target vector and translation in the preceding pose's
horizontal body frame. First account for translation: remaining_forward = forward - dx,
remaining_right = right - dy. The target bearing at that translated position, before this
action's yaw change, is atan2(remaining_right, remaining_forward) * 180 / pi degrees.
Use this bearing to guide dyaw_deg: left is negative and right is positive. A partial
correction is allowed; do not copy the initial bearing into every action's yaw increment.
After applying the yaw increment, express the remaining target vector in the resulting
body frame before planning the next action. This geometric update follows the waypoint
convention; translation and yaw may occur together during execution.
When a large heading correction is needed, reduce translation or use a pure-yaw action
before continuing the approach. Little or no yaw change is appropriate if the target
will remain aligned after the planned translation.
If the remaining horizontal target distance is near zero, do not infer a required turn
from atan2(0, 0). Yaw controls horizontal heading only; it cannot correct vertical framing
or guarantee visibility. Use RGB and camera geometry for actual framing. The remembered
direction does not prove current target visibility.
"""


VISUAL_GOAL_OUTPUT_GUIDANCE = """
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


class VisualGoalTrajectoryPrompt:
    def system_prompt(self):
        return VISUAL_GOAL_SYSTEM_PROMPT

    def build_user_prompt(self, instruction, observation, max_points, *,
                          task_context=None, **execution_context):
        if not isinstance(task_context, VisualGoalPlanningContext):
            raise ProtocolError("visual goal prompt requires typed target context")
        base = build_base_user_prompt(instruction, observation, max_points, **execution_context)
        pending = execution_context.get("pending_trajectory")
        tail = (relative_to_world(pending, observation.vehicle_pose).poses[-1]
                if pending is not None and pending.points else observation.vehicle_pose)
        point = task_context.geometry.world_point
        if not task_context.geometry.reliable or point is None:
            raise ProtocolError("visual goal prompt requires an estimated target position")
        source_time = task_context.target_captured_monotonic
        payload = {
            "target_description": task_context.target_description,
            "keep_target_in_view": task_context.keep_target_in_view,
            "target_point_source": ("bbox_center_depth" if task_context.goal_observation_is_current
                                    else "remembered_bbox_center_depth"),
            "target_observation_id": task_context.target_capture_id or (
                observation.capture_id if task_context.goal_observation_is_current else None),
            "planning_observation_id": observation.capture_id,
            "target_age_at_planning_observation_s": (None if source_time is None else
                round(observation.captured_monotonic-source_time, 4)),
            "coordinate_frame": "horizontal_body_forward_right_down",
            "target_relative_to_observation_body_m": point_in_horizontal_body(point, observation.vehicle_pose),
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
        return user + "\n\n" + VISUAL_GOAL_OUTPUT_GUIDANCE.strip()
