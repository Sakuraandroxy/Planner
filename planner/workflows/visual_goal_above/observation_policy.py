"""Choose useful observation moments and explain the current arrival geometry."""
from planner.capabilities.targeting.above_target_association import project_points
from planner.capabilities.targeting.top_geometry import top_height
from planner.capabilities.completion.visual_goal_above import above_measurements


def observation_target(session):
    destination = session.destination
    if destination is not None:
        z = top_height(session.known_regions,destination.x,destination.y)
        if z is not None:
            return (destination.x,destination.y,z)
    points = session.binding.surface_points if session.binding and session.binding.surface_points else session.surfaces
    return tuple(sum(p[i] for p in points)/len(points) for i in range(3)) if points else None


def needs_observation(session, frame):
    if session.top_view_needed:
        return True
    if session.reference_rgb is None:
        return False
    if not session.target_visible:
        return True
    if session.regions:
        target = observation_target(session)
        return frame is not None and target is not None and not project_points(frame,(target,))
    if session.bbox is not None and session.bbox_frame is not None:
        b,k = session.bbox,session.bbox_frame.intrinsics
        if b.ymin*k.height <= .5 or b.xmin*k.width <= .5 or (1-b.xmax)*k.width <= .5:
            return True
    return False


def record_arrival_state(session, pose, settings, now, trace):
    if now-session.last_status < settings.detection_interval_s:
        return
    inside,height,height_met = above_measurements(pose,session.known_regions,session.height,
        session.height_tolerance_m,session.height_is_minimum)
    reason = ("top_detection_not_started" if not session.top_detection_enabled and not session.regions else
              "top_geometry_unavailable" if not session.regions else
              "outside_bound_roof" if not inside else
              "height_not_reached" if not height_met else "ready_for_completion_observation")
    trace.event("above_arrival_state",pose=pose,projection_inside=inside,
                measured_height_m=height,height_condition_met=height_met,reason=reason,
                destination=session.destination,target_visible=session.target_visible,
                top_detection_enabled=session.top_detection_enabled,reported_top_visible=session.reported_top_visible)
    session.last_status = now
