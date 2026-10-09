from __future__ import annotations

import math

from planner.domain.pose import RelativePoseDelta, WorldPose, wrap_yaw_deg
from planner.domain.trajectory import RelativeTrajectory, WorldTrajectory


def world_pose_delta_in_reference_frame(
    start: WorldPose,
    end: WorldPose,
) -> RelativePoseDelta:
    """Express a measured world-pose change in the start pose horizontal frame."""
    world_dx = end.x - start.x
    world_dy = end.y - start.y
    angle = math.radians(start.yaw_deg)
    return RelativePoseDelta(
        dx=math.cos(angle) * world_dx + math.sin(angle) * world_dy,
        dy=-math.sin(angle) * world_dx + math.cos(angle) * world_dy,
        dz=end.z - start.z,
        dyaw_deg=wrap_yaw_deg(end.yaw_deg - start.yaw_deg),
    )


#将相对航点转为世界坐标的绝对航点
def relative_to_world(trajectory: RelativeTrajectory, start: WorldPose) -> WorldTrajectory:
    """Apply each body-frame delta relative to its preceding predicted pose."""
    x, y, z, yaw = start.x, start.y, start.z, start.yaw_deg
    poses: list[WorldPose] = []
    for point in trajectory.points:
        angle = math.radians(yaw)
        x += math.cos(angle) * point.dx - math.sin(angle) * point.dy
        y += math.sin(angle) * point.dx + math.cos(angle) * point.dy
        z += point.dz
        # Retain small signed commands exactly; wrap_yaw_deg intentionally
        # snaps tiny measured errors to zero and would lose command metadata.
        yaw = (yaw + point.dyaw_deg + 180.0) % 360.0 - 180.0
        poses.append(WorldPose(x=x, y=y, z=z, yaw_deg=yaw))
    return WorldTrajectory(start=start, poses=tuple(poses),
                           yaw_deltas_deg=tuple(point.dyaw_deg for point in trajectory.points))

