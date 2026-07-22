"""Shared trajectory conversion utilities.

Canonical planner output in this project is cumulative body-frame xyz waypoints:
[[dx, dy, dz], ...], relative to the drone pose at planning time.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple


def parse_atomic_action(action: str) -> Tuple[str, float]:
    parts = str(action).strip().split()
    if len(parts) >= 2:
        try:
            return parts[0].lower(), float(parts[1])
        except ValueError:
            return parts[0].lower(), 0.0
    return str(action).strip().lower(), 0.0


def normalize_xyz_waypoints(waypoints: Iterable[Sequence[float]] | None) -> List[List[float]]:
    normalized: List[List[float]] = []
    for wp in waypoints or []:
        if not isinstance(wp, (list, tuple)) or len(wp) < 3:
            continue
        x, y, z = float(wp[0]), float(wp[1]), float(wp[2])
        if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            continue
        normalized.append([round(x, 3), round(y, 3), round(z, 3)])
    return normalized


def actions_to_cumulative_body_waypoints(actions: Iterable[str] | None) -> List[List[float]]:
    """Convert atomic actions to cumulative body-frame xyz waypoints.

    left/right only change the heading used by later forward/backward actions.
    They do not become standalone waypoints because AirSim execution follows
    waypoint positions and turns toward each segment automatically.
    """
    waypoints: List[List[float]] = []
    yaw_deg = 0.0
    x, y, z = 0.0, 0.0, 0.0

    for action in actions or []:
        name, value = parse_atomic_action(action)
        if name == "left":
            yaw_deg -= value
            continue
        if name == "right":
            yaw_deg += value
            continue
        if name in {"forward", "backward"}:
            sign = 1.0 if name == "forward" else -1.0
            rad = math.radians(yaw_deg)
            x += sign * value * math.cos(rad)
            y += sign * value * math.sin(rad)
            waypoints.append([round(x, 3), round(y, 3), round(z, 3)])
            continue
        if name == "up":
            z -= value
            waypoints.append([round(x, 3), round(y, 3), round(z, 3)])
            continue
        if name == "down":
            z += value
            waypoints.append([round(x, 3), round(y, 3), round(z, 3)])

    return waypoints


def cumulative_to_step_body_waypoints(waypoints: Iterable[Sequence[float]] | None) -> List[List[float]]:
    """Convert cumulative body-frame waypoints to adjacent relative displacements."""
    relative: List[List[float]] = []
    prev = [0.0, 0.0, 0.0]
    for wp in normalize_xyz_waypoints(waypoints):
        delta = [
            round(wp[0] - prev[0], 3),
            round(wp[1] - prev[1], 3),
            round(wp[2] - prev[2], 3),
        ]
        if any(abs(v) >= 1e-6 for v in delta):
            relative.append(delta)
        prev = wp
    return relative


def cumulative_body_to_world_positions(
    waypoints: Iterable[Sequence[float]] | None,
    start_pos: Sequence[float],
    start_yaw_deg: float,
    start_rot_body_to_world=None,
) -> List[List[float]]:
    """Convert cumulative body-frame xyz waypoints to absolute world positions."""
    if start_rot_body_to_world is None:
        yaw = math.radians(float(start_yaw_deg))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rot = [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ]
    else:
        rows = [list(row) for row in start_rot_body_to_world]
        if len(rows) != 3 or any(len(row) != 3 for row in rows):
            return cumulative_body_to_world_positions(waypoints, start_pos, start_yaw_deg)
        rot = [[float(v) for v in row] for row in rows]
    sx, sy, sz = float(start_pos[0]), float(start_pos[1]), float(start_pos[2])
    world_positions: List[List[float]] = []

    for x, y, z in normalize_xyz_waypoints(waypoints):
        wx = sx + rot[0][0] * x + rot[0][1] * y + rot[0][2] * z
        wy = sy + rot[1][0] * x + rot[1][1] * y + rot[1][2] * z
        wz = sz + rot[2][0] * x + rot[2][1] * y + rot[2][2] * z
        world_positions.append([round(wx, 3), round(wy, 3), round(wz, 3)])
    return world_positions


def trajectory_delta(waypoints: Iterable[Sequence[float]] | None) -> List[float]:
    cumulative = normalize_xyz_waypoints(waypoints)
    if not cumulative:
        return [0.0, 0.0, 0.0, 0.0]
    end = cumulative[-1]
    return [round(end[0], 3), round(end[1], 3), round(end[2], 3), 0.0]
