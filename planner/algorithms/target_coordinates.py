"""Convert estimated world points into the yaw-only body frame of a pose."""

from __future__ import annotations

import math

from ..domain.pose import WorldPose


def horizontal_distance(
    point: tuple[float, float, float], pose: WorldPose
) -> float:
    return math.hypot(point[0] - pose.x, point[1] - pose.y)


def nearest_surface(
    points: tuple[tuple[float, float, float], ...], pose: WorldPose
) -> tuple[float, float, float]:
    return min(points, key=lambda point: horizontal_distance(point, pose))


def point_in_horizontal_body(
    point: tuple[float, float, float], pose: WorldPose
) -> tuple[float, float, float]:
    """Express a world point as (forward, right, down) in the pose body frame."""
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        raise ValueError("target point must contain three finite coordinates")
    dx, dy, dz = point[0] - pose.x, point[1] - pose.y, point[2] - pose.z
    yaw = math.radians(pose.yaw_deg)
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
        dz,
    )