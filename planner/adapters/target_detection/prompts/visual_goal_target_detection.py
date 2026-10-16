"""Task-specific detection messages with a shared bbox-only output reminder."""


TARGET_DETECTION_OUTPUT_GUIDANCE = """Final response:
Return only [[x1,y1],[x2,y2]], using normalized image-1 coordinates.
If the target cannot be identified, return [].
Do not return a flat array or a JSON object.

Correct output example, format only:
[[0.350,0.450],[0.600,0.750]]"""


class VisualGoalTargetDetectionPrompt:
    def build(self, request):
        if request.reference_rgb is None:
            user = (f"Task: {request.instruction}\nTarget: {request.target_description}\n"
                    "Image 1: current front RGB.\n\n"
                    "Locate this target in image 1.")
        else:
            user = (f"Task: {request.instruction}\n"
                    f"Target: the same previously bound object ({request.target_description}).\n"
                    "Image 1: current front RGB.\nImage 2: bound-target reference crop.\n\n"
                    "Locate that same object in image 1.\n"
                    "Return coordinates relative to the full image 1, not the reference crop.\n"
                    "If the bound object cannot be identified, return []; do not substitute another object.")
        return user + "\n\n" + TARGET_DETECTION_OUTPUT_GUIDANCE
