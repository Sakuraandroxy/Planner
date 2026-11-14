"""Task-specific grounding messages with the bbox and surface-pixel reminder."""


TARGET_DETECTION_OUTPUT_GUIDANCE = """Final response:
Return only {"bbox":[[x1,y1],[x2,y2]],"surface_points":[[x,y],...]}, using normalized full-image-1 coordinates.
Choose up to 5 visible surface points yourself; no fixed positions, symmetry or center point is required.
Each point MUST be inside visible opaque material belonging to this target, never a hole, empty interior, background, glass, or another object.
For open-frame or perforated structures, prefer broad solid components and thick joints; do not sample the empty spaces between members.
Accuracy takes priority over spatial spread and point count. Keep fewer points when uncertain.
Check each final rounded point against image 1 internally. Use at most 4 decimal places.
If the target is identifiable but no suitable surface pixels are clear, keep its bbox and return an empty surface_points array.
If the target cannot be identified, return {"bbox":[],"surface_points":[]}.
Do not return the old bbox-only array, a flat array, extra fields or Markdown.

Correct output example, format only:
{"bbox":[[0.1200,0.3200],[0.6800,0.8600]],"surface_points":[[0.2147,0.6132],[0.4326,0.4819],[0.5813,0.7324]]}"""


class VisualGoalTargetDetectionPrompt:
    def build(self, request):
        if request.reference_rgb is None:
            user = (f"Task: {request.instruction}\nTarget: {request.target_description}\n"
                    "Image 1: current front RGB.\n\n"
                    "Locate this target in image 1.")
        else:
            user = (f"Task: {request.instruction}\n"
                    f"Target: the same object bound in this stage ({request.target_description}).\n"
                    "Image 1: current front RGB.\nImage 2: bound-target reference crop.\n\n"
                    "Locate that same object in image 1.\n"
                    "Return coordinates relative to the full image 1, not the reference crop.\n"
                    'If the bound object cannot be identified, return {"bbox":[],"surface_points":[]}; '
                    'do not substitute another object.')
        return user + "\n\n" + TARGET_DETECTION_OUTPUT_GUIDANCE