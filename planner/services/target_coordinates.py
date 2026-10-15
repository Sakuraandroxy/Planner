"""Convert estimated world points into the yaw-only frame used by waypoints."""
import math

from planner.domain.pose import WorldPose


def point_in_horizontal_body(point: tuple[float, float, float], pose: WorldPose):
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        raise ValueError("target point must contain three finite coordinates")
    dx, dy, dz = point[0]-pose.x, point[1]-pose.y, point[2]-pose.z
    yaw = math.radians(pose.yaw_deg)
    return (math.cos(yaw)*dx+math.sin(yaw)*dy,
            -math.sin(yaw)*dx+math.cos(yaw)*dy, dz)

