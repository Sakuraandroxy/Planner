"""Above completion measurements; no networking or flight commands."""
import numpy as np
from planner.capabilities.targeting.top_geometry import top_height
from planner.domain.visual_goal_above import AboveCompletionRequest


def above_measurements(pose, regions, height, tolerance, minimum):
    z = top_height(regions,pose.x,pose.y)
    measured = z-pose.z if z is not None else None
    met = measured is not None and measured > 0 and (
        measured >= height-tolerance if minimum else abs(measured-height) <= tolerance)
    return z is not None, measured, met


def completion_request(parameters, observation, regions, height, tolerance, hold_confirmed, reference_rgb=None):
    pose = observation.vehicle_pose
    inside, measured, met = above_measurements(pose,regions,height,tolerance,parameters.height_above_m is None)
    world = (pose.x,pose.y,pose.z+measured) if measured is not None else None
    pixel = None
    if world is not None:
        p = np.asarray(observation.rotation_camera_to_world).T @ (np.asarray(world)-observation.camera_position_world)
        if p[0] > 0:
            k = observation.intrinsics
            u,v = (k.fx*p[1]/p[0]+k.cx)/k.width,(k.fy*p[2]/p[0]+k.cy)/k.height
            if 0 <= u <= 1 and 0 <= v <= 1:
                pixel = (float(u),float(v))
    return AboveCompletionRequest(parameters,observation,regions,height,measured,inside,met,
        bool(hold_confirmed),reference_rgb,world,pixel)


def locally_complete(request):
    return request.projection_inside and request.height_condition_met and request.hold_confirmed
