"""ProjectAirSim NED geometry (pure numpy, no simulator import).

ProjectAirSim world and sensor frames are NED: +x forward/north, +y right/east,
+z down. This matches the planner world frame directly, so no axis flips are
needed; yaw is right-positive about +z. Camera topic messages publish the
sensor pose in that world frame, and the configured mount is expressed in the
body frame (forward, right, down) with an RPY orientation.

``DepthGeometry`` consumes ``rotation_camera_to_world`` as the rotation whose
columns are the camera axes (+x optical forward, +y image right, +z image down)
in the world frame, and ``camera_position_world`` as the NED camera origin.
The quaternion published by ProjectAirSim is exactly that camera-to-world
rotation (verified against the configured -3 deg pitch mount).
"""

from __future__ import annotations

import math

import numpy as np

from ...domain.pose import WorldPose, wrap_deg


def _finite(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def quaternion_to_rotation(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    """3x3 camera/body-to-world rotation from an (x, y, z, w) quaternion."""
    values = (x, y, z, w)
    if not all(_finite(value) for value in values):
        raise ValueError("quaternion must be finite")
    norm = math.hypot(x, y, z, w)
    if not math.isclose(norm, 1.0, abs_tol=1e-3, rel_tol=0):
        raise ValueError("quaternion must have unit norm")
    x, y, z, w = (float(value) / norm for value in values)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rpy_rotation_deg(rpy_deg: tuple[float, float, float]) -> np.ndarray:
    """Body-to-world rotation from roll/pitch/yaw degrees (NED axes).

    The quaternion composition matches the reference Planner's AirSim camera
    mount: q = qz(yaw) * qy(pitch) * qx(roll), i.e. roll first, then pitch,
    then yaw about the rotated axes.
    """
    try:
        roll, pitch, yaw = (float(value) for value in rpy_deg)
    except (TypeError, ValueError) as exc:
        raise ValueError("mount rpy must contain three numbers") from exc
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("mount rpy must be finite")
    roll, pitch, yaw = (math.radians(value) / 2 for value in (roll, pitch, yaw))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return quaternion_to_rotation(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotation_yaw_deg(rotation: np.ndarray) -> float:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    return wrap_deg(math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])))


def rotation_from_camera_message(message: dict) -> np.ndarray:
    try:
        return quaternion_to_rotation(
            message["rot_x"], message["rot_y"], message["rot_z"], message["rot_w"]
        )
    except (KeyError, TypeError) as exc:
        raise ValueError("camera message has no usable orientation") from exc


def camera_position_from_message(message: dict) -> tuple[float, float, float]:
    try:
        position = (
            float(message["pos_x"]),
            float(message["pos_y"]),
            float(message["pos_z"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("camera message has no usable position") from exc
    if not all(math.isfinite(value) for value in position):
        raise ValueError("camera message position must be finite")
    return position


def body_pose_from_camera(
    camera_position: tuple[float, float, float],
    camera_rotation: np.ndarray,
    mount_position_body_m: tuple[float, float, float],
    mount_rotation: np.ndarray,
) -> WorldPose:
    """Recover the NED body pose from the published camera pose."""
    rotation = np.asarray(camera_rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("camera rotation must be 3x3")
    offset = np.asarray(mount_position_body_m, dtype=np.float64)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("mount position must contain three finite numbers")
    body_rotation = rotation @ np.asarray(mount_rotation, dtype=np.float64).T
    body_position = np.asarray(camera_position, dtype=np.float64) - body_rotation @ offset
    if not np.isfinite(body_position).all():
        raise ValueError("camera pose produced a non-finite body position")
    return WorldPose(
        float(body_position[0]),
        float(body_position[1]),
        float(body_position[2]),
        rotation_yaw_deg(body_rotation),
    )


def camera_matrix(
    camera_position: tuple[float, float, float], camera_rotation: np.ndarray
) -> np.ndarray:
    """4x4 NED camera-to-world transform (rotation + translation)."""
    rotation = np.asarray(camera_rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("camera rotation must be 3x3")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(camera_position, dtype=np.float64)
    return matrix


def pose_from_kinematics(kinematics: dict) -> WorldPose:
    """World pose from ``GetGroundTruthKinematics`` (NED, metres, degrees)."""
    try:
        position = kinematics["pose"]["position"]
        orientation = kinematics["pose"]["orientation"]
        x = float(position["x"])
        y = float(position["y"])
        z = float(position["z"])
        rotation = quaternion_to_rotation(
            orientation["x"], orientation["y"], orientation["z"], orientation["w"]
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("kinematics response has no usable pose") from exc
    return WorldPose(x, y, z, rotation_yaw_deg(rotation))


def attitude_deg_from_kinematics(
    kinematics: dict,
) -> tuple[float, float, float]:
    """Roll, pitch, yaw degrees from ``GetGroundTruthKinematics`` (NED)."""
    try:
        orientation = kinematics["pose"]["orientation"]
        rotation = quaternion_to_rotation(
            orientation["x"], orientation["y"], orientation["z"], orientation["w"]
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("kinematics response has no usable orientation") from exc
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -rotation[2, 0]))))
    roll = math.degrees(math.atan2(rotation[2, 1], rotation[2, 2]))
    return roll, pitch, rotation_yaw_deg(rotation)