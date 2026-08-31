"""Timestamped Camera API frame schemas shared by runtime and diagnostics."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence


def _point3(value: Sequence[float] | None) -> list[float]:
    values = list(value or [])
    if len(values) < 3:
        return [0.0, 0.0, 0.0]
    return [float(values[0]), float(values[1]), float(values[2])]


def _matrix3(value: Sequence[Sequence[float]] | None) -> list[list[float]]:
    rows = [list(row) for row in list(value or [])]
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return [[float(component) for component in row] for row in rows]


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    horizontal_fov_deg: float
    depth_mode: str = "radial"

    @classmethod
    def from_horizontal_fov(
        cls,
        width: int,
        height: int,
        horizontal_fov_deg: float,
        *,
        depth_mode: str = "radial",
    ) -> "CameraIntrinsics":
        width = max(1, int(width))
        height = max(1, int(height))
        fov = max(1e-3, min(179.0, float(horizontal_fov_deg)))
        fx = float(width) / (2.0 * math.tan(math.radians(fov) * 0.5))
        # AirSim renders square pixels, so fy uses the same focal length in
        # pixel units even when RGB and depth have different aspect ratios.
        return cls(
            width=width,
            height=height,
            fx=fx,
            fy=fx,
            cx=float(width) * 0.5,
            cy=float(height) * 0.5,
            horizontal_fov_deg=fov,
            depth_mode=str(depth_mode or "radial"),
        )

    def to_summary_dict(self) -> dict:
        return {
            "width": int(self.width),
            "height": int(self.height),
            "fx": round(float(self.fx), 6),
            "fy": round(float(self.fy), 6),
            "cx": round(float(self.cx), 6),
            "cy": round(float(self.cy), 6),
            "horizontal_fov_deg": round(float(self.horizontal_fov_deg), 4),
            "depth_mode": self.depth_mode,
        }


@dataclass
class CameraFrame:
    camera_id: str
    capture_id: str
    timestamp_ns: int
    rgb: Any = None
    depth: Any = None
    rgb_png: Optional[bytes] = None
    rgb_jpeg: Optional[bytes] = None
    rgb_intrinsics: Optional[CameraIntrinsics] = None
    depth_intrinsics: Optional[CameraIntrinsics] = None
    camera_position_world: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    camera_quaternion_world: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    rotation_camera_to_world: list[list[float]] = field(
        default_factory=lambda: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    depth_timestamp_ns: int = 0
    depth_camera_position_world: Optional[list[float]] = None
    depth_camera_quaternion_world: Optional[list[float]] = None
    depth_rotation_camera_to_world: Optional[list[list[float]]] = None
    vehicle_position_world: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation_body_to_world: list[list[float]] = field(
        default_factory=lambda: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    navigation_yaw_deg: float = 0.0
    captured_at: float = field(default_factory=time.perf_counter)
    source: str = "camera_api"

    def __post_init__(self) -> None:
        self.camera_id = str(self.camera_id or "camera")
        self.capture_id = str(self.capture_id or "capture")
        self.timestamp_ns = int(self.timestamp_ns or 0)
        self.camera_position_world = _point3(self.camera_position_world)
        self.camera_quaternion_world = [
            float(value) for value in list(self.camera_quaternion_world or [0.0, 0.0, 0.0, 1.0])[:4]
        ]
        self.rotation_camera_to_world = _matrix3(self.rotation_camera_to_world)
        self.vehicle_position_world = _point3(self.vehicle_position_world)
        self.rotation_body_to_world = _matrix3(self.rotation_body_to_world)
        if self.depth_camera_position_world is not None:
            self.depth_camera_position_world = _point3(self.depth_camera_position_world)
        if self.depth_rotation_camera_to_world is not None:
            self.depth_rotation_camera_to_world = _matrix3(self.depth_rotation_camera_to_world)

    @property
    def frame_key(self) -> tuple[str, int, str]:
        return self.camera_id, int(self.timestamp_ns), self.capture_id

    @property
    def rgb_depth_delta_ms(self) -> Optional[float]:
        if self.timestamp_ns <= 0 or self.depth_timestamp_ns <= 0:
            return None
        return abs(float(self.depth_timestamp_ns) - float(self.timestamp_ns)) / 1_000_000.0

    @property
    def optical_axis_world(self) -> list[float]:
        rotation = self.rotation_camera_to_world
        return [float(rotation[0][0]), float(rotation[1][0]), float(rotation[2][0])]

    def to_metadata_dict(self) -> dict:
        rgb_size = list(getattr(self.rgb, "size", []) or [])
        depth_shape = list(getattr(self.depth, "shape", []) or [])
        return {
            "camera_id": self.camera_id,
            "capture_id": self.capture_id,
            "timestamp_ns": int(self.timestamp_ns),
            "depth_timestamp_ns": int(self.depth_timestamp_ns or 0),
            "rgb_depth_delta_ms": self.rgb_depth_delta_ms,
            "rgb_size": rgb_size[:2],
            "depth_shape": depth_shape[:2],
            "camera_position_world": [round(float(v), 6) for v in self.camera_position_world],
            "camera_quaternion_world": [round(float(v), 8) for v in self.camera_quaternion_world],
            "rotation_camera_to_world": [
                [round(float(v), 8) for v in row] for row in self.rotation_camera_to_world
            ],
            "optical_axis_world": [round(float(v), 8) for v in self.optical_axis_world],
            "vehicle_position_world": [round(float(v), 6) for v in self.vehicle_position_world],
            "rotation_body_to_world": [
                [round(float(v), 8) for v in row] for row in self.rotation_body_to_world
            ],
            "navigation_yaw_deg": round(float(self.navigation_yaw_deg), 5),
            "rgb_intrinsics": self.rgb_intrinsics.to_summary_dict() if self.rgb_intrinsics else None,
            "depth_intrinsics": self.depth_intrinsics.to_summary_dict() if self.depth_intrinsics else None,
            "source": self.source,
        }


@dataclass
class MultiCameraSnapshot:
    capture_id: str
    frames: dict[str, CameraFrame]
    vehicle_position_world: list[float]
    rotation_body_to_world: list[list[float]]
    navigation_yaw_deg: float
    captured_at: float = field(default_factory=time.perf_counter)
    source: str = "camera_api"

    def __post_init__(self) -> None:
        self.capture_id = str(self.capture_id or "capture")
        self.frames = {str(key): value for key, value in dict(self.frames or {}).items()}
        self.vehicle_position_world = _point3(self.vehicle_position_world)
        self.rotation_body_to_world = _matrix3(self.rotation_body_to_world)

    def frame(self, camera_id: str) -> Optional[CameraFrame]:
        return self.frames.get(str(camera_id))

    def to_summary_dict(self) -> dict:
        return {
            "capture_id": self.capture_id,
            "camera_ids": list(self.frames),
            "vehicle_position_world": [round(float(v), 6) for v in self.vehicle_position_world],
            "navigation_yaw_deg": round(float(self.navigation_yaw_deg), 5),
            "captured_at": float(self.captured_at),
            "source": self.source,
        }


@dataclass(frozen=True)
class WorldRay:
    camera_id: str
    capture_id: str
    origin_world: tuple[float, float, float]
    direction_world: tuple[float, float, float]
    timestamp_ns: int
    angular_uncertainty_deg: float = 0.0

