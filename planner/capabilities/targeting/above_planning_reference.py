"""Coordinate arithmetic for a prompt example, never an execution decision."""
import math

from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.services.trajectory_transform import world_pose_delta_in_reference_frame


def continuation_reference(start, destination, look_at, camera_heading_offset, *,
                           step_m, yaw_step_deg):
    pose, actions = start, []
    for _ in range(5):
        vector = (destination.x-pose.x,destination.y-pose.y,destination.z-pose.z)
        distance = math.hypot(*vector)
        scale = min(1.,step_m/distance) if distance else 0.
        x,y,z = (value+delta*scale for value,delta in zip((pose.x,pose.y,pose.z),vector))
        yaw = pose.yaw_deg
        if look_at is not None and camera_heading_offset is not None:
            dx,dy = look_at[0]-x,look_at[1]-y
            if math.hypot(dx,dy) > 1e-6:
                wanted = math.degrees(math.atan2(dy,dx))-camera_heading_offset
                remaining = wrap_yaw_deg(wanted-pose.yaw_deg)
                yaw = wrap_yaw_deg(pose.yaw_deg+max(-yaw_step_deg,min(yaw_step_deg,remaining)))
        end = WorldPose(x,y,z,yaw)
        delta = world_pose_delta_in_reference_frame(pose,end)
        actions.append([delta.dx,delta.dy,delta.dz,delta.dyaw_deg])
        pose = end
    return actions
