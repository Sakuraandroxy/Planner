"""Task-specific grounding messages with the bbox and surface-pixel reminder."""

from .render import load_fragment, render

TARGET_DETECTION_OUTPUT_GUIDANCE = load_fragment("output_guidance")


class VisualGoalTargetDetectionPrompt:
    def build(self, request):
        template = (
            "user_prompt" if request.reference_rgb is None else "user_prompt_with_reference"
        )
        return (
            render(
                load_fragment(template),
                instruction=request.instruction,
                target_description=request.target_description,
            )
            + "\n\n"
            + TARGET_DETECTION_OUTPUT_GUIDANCE
        )
