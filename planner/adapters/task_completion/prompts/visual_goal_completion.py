"""Completion-only messages; never used by the trajectory planner."""

from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.adapters.prompts import completion_values, render_prompt
from planner.adapters.task_completion.prompt import CompletionMessages
from planner.domain import VisualGoalCompletionInput
from planner.workflows.visual_goal_navigation import require_visual_completion_evidence


class VisualGoalCompletionPrompt:
    @staticmethod
    def prepare(request: VisualGoalCompletionInput) -> CompletionMessages:
        require_visual_completion_evidence(request)
        images = (
            (request.observation.rgb, "JPEG"),
            (aligned_depth_image(request.observation), "PNG"),
        )
        if request.reference_rgb is not None:
            images += ((request.reference_rgb, "JPEG"),)
        return CompletionMessages(
            VisualGoalCompletionPrompt.system_prompt(),
            VisualGoalCompletionPrompt.build_user_prompt(request),
            images,
        )

    @staticmethod
    def system_prompt():
        return render_prompt("completion_system")

    @staticmethod
    def build_user_prompt(request: VisualGoalCompletionInput):
        return render_prompt("completion", **completion_values(request))
