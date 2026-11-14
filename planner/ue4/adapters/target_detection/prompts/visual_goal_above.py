"""Only visible target top surfaces; coordinates share the standard image origin."""
import json
from dataclasses import asdict
SYSTEM = """Locate the specified static target in image 1. Return ONLY one JSON object with exactly
bbox, surface_points, top_regions. Coordinates are normalized to [0,1]: origin at image top-left,
x increases right, y increases down. bbox is [[xmin,ymin],[xmax,ymax]].
Fit bbox tightly to the specified target's visible extent, especially its upper silhouette.
Exclude sky, foreground pavement and neighboring objects. If the target is clipped, end the box
at the image edge; do not invent its off-image extent. The box's upper edge is not a roof surface.
surface_points contains 0 to 5 points strictly on visible material of this target, not holes,
background, reflections or silhouette boundaries. Choose suitable points yourself.
top_regions contains the VISIBLE TOP surfaces of this target, not facade or image-upper regions.
Each region is {"outline":[[x,y],...],"holes":[[[x,y],...],...],"support_points":[[x,y],...]}. Each ring has at least 3 vertices,
implicitly closed. Preserve concavities, openings, separate levels and separate visible surfaces.
The request selects either target_only or target_and_top. For target_only, locate the object
and its outer surface points only. Always return top_regions=[] in that mode, including when
you think a roof is visible. Do not estimate a hidden roof or the space above the target.
For target_and_top, return visible upward-facing target material. Upper facade, the front of
an eave or parapet, treetops and a roof outline without visible top material are not roof regions.
Return only the visible portion; do not invent hidden roof extent. A visible facade without a
visible top has top_regions=[]. Each region needs 3 to 5 well-spread, non-collinear support_points
strictly on THAT visible roof surface, not a wall, trees, sky, holes or roof edges. Separate roof
slopes/levels into separate regions. Trace the actual roof material, not a loose envelope around
trees above it. Outer surface_points may lie on other target material; roof support_points may not.
Each region is approximated locally by one plane using its support depths. Split distinct slopes
and levels; keep the outline on that same visible roof surface. Do not include a wall in a roof region.
If the target is absent return
{"bbox":[],"surface_points":[],"top_regions":[]}.
Image 2, when supplied, is an earlier reference of the SAME target, not another current view.
Only locate that same target in image 1. The camera may now face upward, obliquely, or downward.
Once a reference is supplied, directional words such as front-right describe the INITIAL selection,
not a new selection in the current view. Do not switch to another roof in that direction.
Magenta numbered marks, when present, project previously measured points of the bound target.
They are location predictions and may be occluded or imperfect. They are not surface evidence.
Use the reference appearance and the known world location together. Exclude neighboring buildings,
even when they share the bbox. Return an empty target if the bound target is not visible.
Correct format example:
{"bbox":[[0.1,0.2],[0.9,0.9]],"surface_points":[[0.4,0.6]],"top_regions":[{"outline":[[0.2,0.3],[0.7,0.3],[0.7,0.5],[0.2,0.5]],"holes":[],"support_points":[[0.3,0.35],[0.6,0.35],[0.45,0.45]]}]}.
Do not output explanations, Markdown, null, or additional fields. Do not copy example coordinates.
"""


class VisualGoalAboveTargetDetectionPrompt:
    def build(self, request):
        return (f"Task: {request.instruction}\nTarget: {request.target_description}\n"
                f"Detection mode: {request.mode.value}\n"
                "Image 1: current RGB from the single UAV camera. Follow the selected detection mode.\n"
                + ("Image 2: initial reference of the bound target. Keep this identity.\n" if request.reference_rgb is not None else "")
                + "Numbered bound-target predictions in image 1 (world NED and normalized image x,y): "
                + json.dumps([asdict(p) for p in request.projected_target_points])
                + "\nBound-target measured world points [NED x,y,z]: "
                + json.dumps(request.bound_target_points_world))
