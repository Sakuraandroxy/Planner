from planner.adapters.prompts import render_prompt
from planner.adapters.prompts.values import vehicle_values
from planner.adapters.prompts.values import visual_goal_values
from planner.domain import VisualGoalPlanningContext
from planner.errors import ProtocolError
from planner.algorithms.depth import FreeSpaceProfileSettings
from planner.algorithms.depth import measure_free_space

from .base import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    build_aerial_base_user_prompt,
    build_base_user_prompt,
)
from .free_navigation import format_free_space_profile


def visual_goal_system_prompt(*, aerial: bool = False) -> str:
    """Return the UAV or ground-vehicle visual-goal system prompt."""
    base = AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT if aerial else BASE_TRAJECTORY_SYSTEM_PROMPT
    return render_prompt(
        "visual_goal_navigation", base_system=base.rstrip("\n"), **vehicle_values(aerial)
    )


VISUAL_GOAL_SYSTEM_PROMPT = visual_goal_system_prompt()


# UAV variant, ported from the reference Planner (/roxy/zhusai/Planner).
# The drone keeps vertical motion; the rover above keeps carla-vln's
# ground-vehicle rules. Both variants add the project's target-side dyaw
# constraint, which is not present in either reference.
AERIAL_VISUAL_GOAL_SYSTEM_PROMPT = visual_goal_system_prompt(aerial=True)


VISUAL_GOAL_VIEW_GUIDANCE = render_prompt("visual_goal_view_guidance")

VISUAL_GOAL_OUTPUT_GUIDANCE = render_prompt("visual_goal_output", **vehicle_values(False))
AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE = render_prompt(
    "visual_goal_output", **vehicle_values(True)
)


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
        if not task_context.geometry.reliable or task_context.geometry.world_point is None:
            raise ProtocolError("visual goal prompt requires an estimated target position")
        base = (build_aerial_base_user_prompt if self.aerial else build_base_user_prompt)(
            instruction, observation, max_points, **execution_context)
        # Ground vehicle only: give the model measured metric obstacle
        # distances instead of leaving it to estimate scale from the image.
        profile = (
            ""
            if self.aerial
            else format_free_space_profile(
                measure_free_space(observation, self.free_space)
            ).rstrip("\n")
        )
        values = visual_goal_values(
            observation,
            task_context,
            execution_context.get("pending_trajectory"),
        )
        if values["target_age_s"] is not None and values["target_age_s"] < 0:
            raise ProtocolError("target estimate is newer than the planning observation")
        output = (
            AERIAL_VISUAL_GOAL_OUTPUT_GUIDANCE
            if self.aerial
            else VISUAL_GOAL_OUTPUT_GUIDANCE
        )
        return render_prompt(
            "visual_goal_navigation_user",
            base_user=base.rstrip("\n"),
            free_space_profile=profile,
            view_guidance=(
                VISUAL_GOAL_VIEW_GUIDANCE.rstrip("\n")
                if task_context.keep_target_in_view
                else ""
            ),
            output_guidance=output.rstrip("\n"),
            **values,
        )
