import json

from planner.domain.pose import WorldPose
from planner.domain.visual_goal import GoalReference


GOAL_OBSERVER_SYSTEM_PROMPT = """Locate a static UAV navigation target in the CURRENT RGB image.
Return JSON only with protocol_version="goal_observation_v2", status visible|not_visible|ambiguous,
bbox [xmin,ymin,xmax,ymax], surface_candidates, description, evidence, and same_target boolean.
Each surface_candidates item has exactly candidate_id (nonempty string), point [u,v],
description (surface region), and matches_reference_region (boolean).
All coordinates are normalized to [0,1] in the CURRENT full image; never use pixel integers.
For visible, provide 3-5 dispersed points when evidence supports them, subject to the user
candidate limit. Fewer points, including one, are valid; do not invent points to meet a quota.
Candidate IDs are unique within this response only, not persistent object/region IDs.
Mark points INSIDE the requested object's visible solid surface, not sky,
background, a foreground occluder, or the center of an empty opening.
For not_visible or ambiguous set bbox=null and surface_candidates=[].
Do not invent unseen targets. Multiple indistinguishable candidates are ambiguous.
Rain, snow, reflections and fog artifacts are not targets. Report insufficient evidence honestly.
Do not estimate meters or claim task completion.
Image 2, when supplied, is the object reference; image 3 is the selected region reference.
The user specifies the original selected point normalized WITHIN image 3 for correspondence.
same_target describes SAME OBJECT identity only; set false if uncertain.
At most one candidate may match the selected reference location: repeat that location as
closely as visible evidence allows and set matches_reference_region=true only for it.
Other points on the same building are not a region match. If the region is no longer visible
or correspondence is uncertain, set all region matches false; do not silently choose another facade.
With no reference, all region matches must be false. Camera heading does not change object identity.
References are not current images: return coordinates in image 1 only.
Directions in the original description are anchored to the initial task heading,
not reinterpreted at every new camera heading."""


def goal_user_prompt(description: str, current_pose: WorldPose,
                     initial_pose: WorldPose | None, reference: GoalReference | None,
                     maximum_candidates: int = 5) -> str:
    return json.dumps({
        "target_description": description,
        "maximum_surface_candidates": maximum_candidates,
        "initial_heading_deg": (initial_pose or current_pose).yaw_deg,
        "current_vehicle_heading_deg": current_pose.yaw_deg,
        "reference": None if reference is None else {
            "description": reference.description,
            "selected_world_anchor": reference.world_point,
            "selected_region_description": reference.region_description,
            "selected_point_in_image_3": reference.region_point,
            "image_2_is_reference_crop": True,
            "image_3_is_selected_region_crop": True,
        },
    }, ensure_ascii=False)
