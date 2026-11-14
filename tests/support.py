"""Shared helpers for hand-built test observations (not collected by pytest)."""

from __future__ import annotations

import math

import numpy as np


def ue_camera_matrix(
    x_units: float, y_units: float, z_units: float, yaw_deg: float
) -> np.ndarray:
    """4x4 camera-to-UE-world matrix (row-major) for a level, yawed camera.

    UE yaw is right-positive, so the rotation maps forward (1,0,0) to
    (cos, sin, 0). Test observations pass ``(x, y, -z, yaw)`` so their default
    ``camera_position_world`` equals the planner pose.
    """
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [c, -s, 0.0, x_units],
            [s, c, 0.0, y_units],
            [0.0, 0.0, 1.0, z_units],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )