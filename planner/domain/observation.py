from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .pose import WorldPose


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    horizontal_fov_deg: float


@dataclass(frozen=True)
class Observation:
    rgb: Any
    depth_meters: Any
    depth_image: Any
    vehicle_pose: WorldPose
    camera_id: str
    intrinsics: CameraIntrinsics
    camera_position_world: tuple[float, float, float]
    rotation_camera_to_world: tuple[tuple[float, float, float], ...] #3x3 rotation matrix
    timestamp_ns: int
    depth_intrinsics: CameraIntrinsics | None = None
    depth_kind: str = "unknown"
    captured_monotonic: float = 0.0
    capture_id: str = ""
    depth_timestamp_ns: int = 0
    stopped: bool = False
    capture_started_monotonic: float = 0.0
    capture_finished_monotonic: float = 0.0

