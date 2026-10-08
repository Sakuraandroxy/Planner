import json

from planner.domain.pose import WorldPose
from planner.domain.visual_goal import GoalReference


GOAL_OBSERVER_SYSTEM_PROMPT = """Locate a static UAV navigation target in the CURRENT RGB image.
Return JSON only with protocol goal_observation_v1, status visible|not_visible|ambiguous,
bbox [xmin,ymin,xmax,ymax], surface_point [u,v], description, evidence, and same_target boolean.
All coordinates are normalized to [0,1] in the CURRENT full image; never use pixel integers.
For visible, mark a point INSIDE the requested object's visible solid surface, not sky,
background, a foreground occluder, or the center of an empty opening.
For not_visible or ambiguous set bbox and surface_point to null.
Do not invent unseen targets. Multiple indistinguishable candidates are ambiguous.
Rain, snow, reflections and fog artifacts are not targets. Report insufficient evidence honestly.
Do not estimate meters or claim task completion.
If a reference crop is supplied, match the SAME object and the same surface region;
changing camera heading does not change target identity. Set same_target=false if uncertain.
The reference crop is not the current image: return coordinates in image 1 only.
Directions in the original description are anchored to the initial task heading,
not reinterpreted at every new camera heading."""


def goal_user_prompt(description: str, current_pose: WorldPose,
                     initial_pose: WorldPose | None, reference: GoalReference | None) -> str:
    return json.dumps({
        "target_description": description,
        "initial_heading_deg": (initial_pose or current_pose).yaw_deg,
        "current_vehicle_heading_deg": current_pose.yaw_deg,
        "reference": None if reference is None else {
            "description": reference.description,
            "last_world_surface_point": reference.world_point,
            "image_2_is_reference_crop": True,
        },
    }, ensure_ascii=False)
