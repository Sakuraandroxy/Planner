"""Detection-only box protocol, independent of planning and completion prompts."""
TARGET_DETECTION_SYSTEM_PROMPT = """Locate the specified target in image 1.
Return only its tight visible bounding box as [[x1,y1],[x2,y2]], ordered top-left then bottom-right.
Use normalized coordinates in [0,1]: origin at the top-left image boundary, x rightward, y downward. Use at most 3 decimal places.
With a reference image, match the same target in image 1 without reselecting by current image order.
Return a partial visible box if identifiable. Never substitute another object when the bound target leaves view.
If the target cannot be identified, return []. No explanations, code fences or extra fields."""
