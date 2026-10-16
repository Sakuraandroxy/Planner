"""Detection-only box protocol, independent of planning and completion prompts."""
TARGET_DETECTION_SYSTEM_PROMPT = """Locate the specified target in image 1.

Return exactly ONE of these two JSON formats:
- Target identified: [[x1,y1],[x2,y2]]
- Target not identifiable: []

For a detected target:
- Return exactly two corners: top-left, then bottom-right.
- Each corner contains exactly two numbers.
- Use normalized coordinates in [0,1], relative to the full image 1.
- The origin is the top-left image boundary; x increases rightward and y increases downward.
- Require x1 < x2 and y1 < y2. Use at most 3 decimal places.
- Return one tight visible bounding box for the specified target.
- If partially visible but identifiable, return its visible bounding box.
- With a reference image, match the same object; do not select another object by current image order.

Output only the JSON array on one line.
Do not output an object, field names, a flat four-number array, multiple boxes,
null, strings, explanations, Markdown, or code fences.

Correct detected-target example:
[[0.350,0.450],[0.600,0.750]]

Correct not-identifiable example:
[]

Examples demonstrate format only. Do not copy their coordinates."""
