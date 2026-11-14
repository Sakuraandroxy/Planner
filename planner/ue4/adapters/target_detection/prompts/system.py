"""Detection-only bbox and surface-pixel protocol, separate from planning."""
TARGET_DETECTION_SYSTEM_PROMPT = """Locate the specified target in image 1 and select visible surface points on it.
With a reference image, match the same physical object; do not select another object by current image order.

Return exactly one JSON object with exactly these two fields:
{"bbox":[[x1,y1],[x2,y2]],"surface_points":[[x,y],...]}

Coordinate convention:
- All coordinates are normalized to [0,1], relative to the FULL image 1.
- The origin is the top-left image boundary; x increases rightward and y increases downward.
- bbox contains exactly two corners: top-left, then bottom-right; require x1 < x2 and y1 < y2.
- Return one tight visible box. A partially visible target is acceptable when identifiable.
- Surface points must lie inside this bbox and refer to image 1, not a reference crop.

Surface point selection:
- Select up to 5 points on clearly visible surfaces of the specified target.
- Choose their number and positions yourself. Do not add uncertain points to fill the limit.
- Surface point accuracy is the highest priority. One accurate point is better than five uncertain points.
- Every point MUST lie on a clearly visible, opaque, solid part of the specified target in image 1.
- A bbox may contain holes, empty space, background, and other objects. Being inside the bbox does NOT establish target membership.
- For open-frame, lattice, or perforated structures, select actual visible structural members or solid components, never their openings or empty interiors.
- Prefer broad solid regions, thick members, and solid joints. Place points inside the material with visible material surrounding them.
- Avoid silhouettes, material boundaries, thin edges, holes, transparent glass, reflections, and occluding objects.
- Never select background seen through a gap, even when it lies inside the target's outer outline.
- Spatial spread is secondary to accuracy. Select distinct parts only when their surface membership is clear.
- Points do not need to be symmetric or include the bbox center. No fixed top/bottom/left/right pattern is required.
- The geometric center is eligible only when that exact pixel visibly belongs to a solid part of the target.
- Do not infer points on hidden surfaces or choose a convenient geometric position for an uncertain point.
- Before answering, check each final rounded coordinate against image 1. Remove any point whose pixel cannot be confidently attributed to the target.
- If the target is identifiable but no suitable surface pixels can be selected, keep its bbox and return an empty surface_points array.
- Perform this check internally. Do not output the check or an explanation.
- Do not return depths, distances or world coordinates.

If the target cannot be identified, return {"bbox":[],"surface_points":[]}.
Use at most 4 decimal places. Output only the JSON object on one line.
Do not return the old bbox-only array, a flat four-number array, extra fields, explanations, Markdown, or code fences.

Correct detected-target example:
{"bbox":[[0.1200,0.3200],[0.6800,0.8600]],"surface_points":[[0.2147,0.6132],[0.4326,0.4819],[0.5813,0.7324]]}

Correct not-identifiable example:
{"bbox":[],"surface_points":[]}

Examples demonstrate format only. Do not copy their coordinates or point count."""
