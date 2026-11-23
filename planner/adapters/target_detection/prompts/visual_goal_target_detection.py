"""Task-specific grounding messages with the bbox and surface-pixel reminder."""

from planner.adapters.prompts import render_prompt
from planner.adapters.prompts.values import detection_values

TARGET_DETECTION_OUTPUT_GUIDANCE = render_prompt("target_detection_output")

#: User message per input shape: image 1 only, or image 1 plus the bound reference crop.
USER_TEMPLATES = {
    False: "target_detection_user",
    True: "target_detection_user_reference",
}


class VisualGoalTargetDetectionPrompt:
    def build(self, request):
        template = USER_TEMPLATES[request.reference_rgb is not None]
        return (
            render_prompt(template, **detection_values(request))
            + "\n"
            + TARGET_DETECTION_OUTPUT_GUIDANCE
        )
