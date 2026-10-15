import json
from dataclasses import asdict
from planner.domain.visual_goal import VisualGoalPlanningContext
from planner.errors import ProtocolError

from .base import BASE_TRAJECTORY_SYSTEM_PROMPT, build_base_user_prompt


VISUAL_GOAL_SYSTEM_PROMPT = BASE_TRAJECTORY_SYSTEM_PROMPT + """

Static visual goal approach rules:
The objective is to enter arrival_radius_m around the target's estimated position and hover.
Any position inside that radius is acceptable; do not aim for its exact boundary.
Use distance_uncertainty_margin_m for the arrival estimate. Preserve the selected object's identity.
The target point is the current RGB bbox center projected using raw depth, not its true 3D center
or a persistent physical landmark. Its estimate is refreshed at each target observation.
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
When goal_observation_is_current=false, the target context is a previously bound static
reference, not a fresh detection or depth measurement; current-image bbox/point are omitted.
Use the last target position and current RGB for continuation. Completion is judged separately.
"""


class VisualGoalTrajectoryPrompt:
    def system_prompt(self):
        return VISUAL_GOAL_SYSTEM_PROMPT

    def build_user_prompt(self, instruction, observation, max_points, *,
                          task_context=None, **execution_context):
        if not isinstance(task_context, VisualGoalPlanningContext):
            raise ProtocolError("visual goal prompt requires typed target context")
        base = build_base_user_prompt(instruction, observation, max_points, **execution_context)
        return base + "\nVisual goal context (estimated evidence, not target ground truth):\n" + json.dumps(
            asdict(task_context), ensure_ascii=False, separators=(",", ":"),
        )
