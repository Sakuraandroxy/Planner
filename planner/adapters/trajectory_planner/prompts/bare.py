"""Instruction-only trajectory prompt for the bare VLN profile.

The staged prompts in ``base.py`` and ``visual_goal_navigation.py`` render the
world pose, the target coordinates, the camera geometry, the unexecuted pending
queue and the execution history. This builder renders the original instruction
and the output format only: the profile demonstrates what the model does with
the instruction and the two images, so every other input is withheld.
"""

BARE_NAVIGATION_SYSTEM_PROMPT = """You are the trajectory planner for an autonomous vehicle.

You receive two images of the current view and the original navigation instruction.
Image 1 is the current front RGB image.
Image 2 is the aligned front depth visualization: brighter pixels are nearer surfaces.

Plan the next short part of the instruction from the current view.
Preserve the intent and action order of the instruction.
Do not invent landmarks, objects, metric distances, or free space that the images do not show.
{motion_rule}

Return relative increments, not absolute positions or absolute headings.
Each waypoint is [dx, dy, dz, dyaw_deg]: translation in metres and yaw change in
degrees, both relative to the preceding waypoint.
+dx is forward and +dy is right. Positive dyaw_deg turns right, negative turns left.
Use finite numbers and keep each dyaw_deg within [-180, 180].

Mandatory trajectory output format:
Return exactly one JSON array of waypoint arrays.
Each waypoint array must contain exactly 4 finite JSON numbers:
[dx, dy, dz, dyaw_deg].
Return as many waypoints as this part of the instruction needs; the count is not fixed.
Enclose every waypoint array in ONE outer array.
Return the entire result on one line, starting with [[ and ending with ]].

Correct output example:
The example shows the format only; choose the actual waypoints from the current view.
{example}

Output only the final numerical trajectory.
Do not output scene descriptions, calculations, reasoning, action descriptions,
introductions, Markdown, code fences, comments, field names, or completion status.
Do not output placeholders, ellipses, strings, booleans, null, NaN, or Infinity.
"""

_GROUND_MOTION_RULE = """This ground vehicle cannot change height: output dz = 0 for every waypoint.
Do not plan climbing or descending."""
_AERIAL_MOTION_RULE = """AirSim uses world NED coordinates for the translation:
+dz is DOWN and -dz is UP. Upward and downward motion are available."""

_GROUND_OUTPUT_EXAMPLE = "[[0.8,0.0,0.0,0.0],[0.9,0.1,0.0,4.0],[0.7,0.0,0.0,0.0]]"
_AERIAL_OUTPUT_EXAMPLE = "[[0.8,0.0,-0.2,0.0],[0.9,0.1,0.0,4.0],[0.7,0.0,0.1,0.0]]"


def bare_system_prompt(*, aerial: bool = False) -> str:
    """Return the bare system prompt for the UAV or the ground vehicle."""
    return BARE_NAVIGATION_SYSTEM_PROMPT.format(
        motion_rule=_AERIAL_MOTION_RULE if aerial else _GROUND_MOTION_RULE,
        example=_AERIAL_OUTPUT_EXAMPLE if aerial else _GROUND_OUTPUT_EXAMPLE,
    )


def build_bare_user_prompt(instruction: str) -> str:
    """Render the instruction verbatim; no environment state is added."""
    text = str(instruction or "").strip()
    if not text:
        raise ValueError("bare trajectory prompt requires an instruction")
    return "\n".join([
        "Visual inputs:",
        "- Image 1 is the current front RGB image.",
        "- Image 2 is the aligned front depth visualization.",
        "- Brighter depth pixels mean nearer surfaces.",
        "",
        "Original navigation instruction:",
        text,
        "",
        "Return the next trajectory as one JSON array of [dx, dy, dz, dyaw_deg] waypoints.",
    ])


class BareTrajectoryPrompt:
    """Trajectory prompt that keeps the instruction and the output format only.

    The signature matches ``TrajectoryPromptBuilder`` so the planner adapter can
    inject it unchanged. Execution context arguments are accepted and ignored on
    purpose: withholding them is the contract of this profile, not an oversight.
    """

    def __init__(self, *, aerial: bool = False):
        self.aerial = bool(aerial)

    def system_prompt(self) -> str:
        return bare_system_prompt(aerial=self.aerial)

    def build_user_prompt(self, instruction, observation, max_points, *,
                          current_guidance=None, guidance_start=None, round_index=1,
                          initial_pose=None, previous_trajectory=None,
                          previous_trajectory_start=None, pending_trajectory=None,
                          task_context=None) -> str:
        return build_bare_user_prompt(instruction)
