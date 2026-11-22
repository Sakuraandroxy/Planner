"""越点判据：距离圈与越点走廊，两者任一成立且偏航对齐（裁决 R6）。"""

from __future__ import annotations

import math

from planner.domain import QueuedWaypoint, WorldPose


def reached(
    waypoint: QueuedWaypoint,
    pose: WorldPose,
    unwrapped_yaw: float,
    goal_yaw: float,
    position_tolerance: float,
    yaw_tolerance: float,
    passed_lateral_tolerance: float | None = None,
) -> bool:
    a, b = waypoint.start, waypoint.end
    delta = (b.x-a.x, b.y-a.y, b.z-a.z)
    offset = (pose.x-a.x, pose.y-a.y, pose.z-a.z)
    norm_sq = sum(v*v for v in delta)
    distance = math.dist((pose.x, pose.y, pose.z), (b.x, b.y, b.z))
    # The "passed" corridor is wider than the reach ball for non-holonomic
    # vehicles: overshooting an endpoint with lateral drift should advance the
    # queue, not send the car back to hit a point it cannot strafe onto.
    corridor = position_tolerance if passed_lateral_tolerance is None else passed_lateral_tolerance
    passed = False
    if norm_sq > 1e-12:
        projection = sum(x*y for x, y in zip(offset, delta))/norm_sq
        lateral = math.sqrt(sum((x-projection*y)**2 for x, y in zip(offset, delta)))
        passed = projection >= 1 and lateral <= corridor
    return ((distance <= position_tolerance or passed) and abs(goal_yaw-unwrapped_yaw) <= yaw_tolerance)
