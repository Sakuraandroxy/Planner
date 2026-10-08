import json
from dataclasses import asdict

from .base import BASE_TRAJECTORY_SYSTEM_PROMPT, build_base_user_prompt


VISUAL_GOAL_SYSTEM_PROMPT = BASE_TRAJECTORY_SYSTEM_PROMPT + """

Static visual goal approach rules:
The objective is to approach the selected static visible surface and hover at the given
standoff distance, not collide with its surface or aim for an unobserved building center.
Target identity/direction is anchored at stage start. Preserve the selected object.
Use measured target geometry as evidence, not as guaranteed ground truth. Plan useful
progress when the corridor is clear, reduce motion near the arrival band, avoid overshooting.
Keep world height near stage-start height. Prefer forward motion; no blind lateral/backward flight.
Every complete segment must fit both execution distance and smooth-duration limits.
If current view cannot support a turn with translation, output a pure yaw point first;
after a large heading change the executor reobserves and discards unsupported later points.
Previous executed command prefix is not an exact measured flight path; use actual pose updates.
Do not output completion status: local depth/identity/stability checks determine completion.
"""


class VisualGoalTrajectoryPrompt:
    supports_visual_goal_context = True

    def system_prompt(self):
        return VISUAL_GOAL_SYSTEM_PROMPT

    def build_user_prompt(self, instruction, observation, max_points, *,
                          visual_goal_context=None, **execution_context):
        if visual_goal_context is None:
            raise ValueError("visual goal prompt requires typed target context")
        base = build_base_user_prompt(instruction, observation, max_points, **execution_context)
        return base + "\nVisual goal context (estimated evidence, not target ground truth):\n" + json.dumps(
            asdict(visual_goal_context), ensure_ascii=False, separators=(",", ":"),
        )
