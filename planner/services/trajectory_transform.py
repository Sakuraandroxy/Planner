from __future__ import annotations

import math

from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import RelativeTrajectory, WorldTrajectory


def relative_to_world(trajectory: RelativeTrajectory, start: WorldPose) -> WorldTrajectory:
    """Apply each body-frame delta relative to its preceding predicted pose."""
    x, y, z, yaw = start.x, start.y, start.z, start.yaw_deg
    poses: list[WorldPose] = []
    for point in trajectory.points:
        angle = math.radians(yaw)
        x += math.cos(angle) * point.dx - math.sin(angle) * point.dy
        y += math.sin(angle) * point.dx + math.cos(angle) * point.dy
        z += point.dz
        yaw = wrap_yaw_deg(yaw + point.dyaw_deg)
        poses.append(WorldPose(x=x, y=y, z=z, yaw_deg=yaw))
    return WorldTrajectory(start=start, poses=tuple(poses))

