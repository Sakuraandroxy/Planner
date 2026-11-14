"""Observation and camera types shared across adapters."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

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

    @classmethod
    def from_horizontal_fov(
        cls, width: int, height: int, hfov_deg: float
    ) -> "CameraIntrinsics":
        """Build a pinhole model from CARLA's horizontal field of view.

        CARLA renders square pixels, so fx == fy. The principal point uses
        half-pixel centres, matching typical raster back-projection.
        """
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive")
        hfov = math.radians(hfov_deg)
        if not 0.0 < hfov < math.pi:
            raise ValueError("horizontal FOV must be in (0, 180) degrees")
        fx = (width / 2.0) / math.tan(hfov / 2.0)
        return cls(
            width=width,
            height=height,
            fx=fx,
            fy=fx,
            cx=width / 2.0 - 0.5,
            cy=height / 2.0 - 0.5,
            horizontal_fov_deg=float(hfov_deg),
        )


@dataclass(frozen=True)
class Observation:
    """Stopped, synchronized RGB + depth observation.

    ``depth_m`` follows ``depth_kind``: optical-axis depth (metres along the
    camera forward axis) by default, or Euclidean ray range when the rig is
    calibrated with ``camera.depth_is_range``. The camera matrix is a 4x4 UE
    camera-to-world transform in CARLA world units (metres in the metre-scaled
    build used here).
    """

    frame: int
    timestamp_s: float
    pose: WorldPose
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: CameraIntrinsics
    camera_matrix: np.ndarray
    camera_id: str = ""
    timestamp_ns: int = 0
    captured_monotonic: float = 0.0
    capture_id: str = ""
    stopped: bool = False
    # Optional single-channel visualisation image (same mapping as the model
    # prompt); ``depth_m`` remains the calibrated metric source of truth.
    depth_image: np.ndarray | None = None
    # Reference-consumed camera evidence. CARLA renders RGB and depth from one
    # parentless camera, so both share intrinsics; the rig fills these fields
    # explicitly. Defaults derive them for hand-built test observations.
    depth_kind: str = "optical_depth"
    depth_timestamp_ns: int = 0
    camera_position_world: tuple[float, float, float] | None = None
    rotation_camera_to_world: np.ndarray | None = None
    depth_intrinsics: CameraIntrinsics | None = None
    # Depth visualization protocol shipped to the model prompts.
    depth_mapping: str = "log_inverse_gray_v1"
    depth_min_m: float = 0.5
    depth_max_m: float = 200.0

    def __post_init__(self) -> None:
        if self.rgb.ndim != 3 or self.rgb.shape[2] != 3:
            raise ValueError("rgb must be HxWx3")
        if self.depth_m.shape != self.rgb.shape[:2]:
            raise ValueError("depth shape must match rgb")
        if self.depth_image is not None and self.depth_image.shape[:2] != self.rgb.shape[:2]:
            raise ValueError("depth_image shape must match rgb")
        if self.camera_matrix.shape != (4, 4):
            raise ValueError("camera_matrix must be 4x4")
        if self.depth_kind not in ("optical_depth", "ray_range"):
            raise ValueError("depth_kind must be optical_depth or ray_range")
        matrix = np.asarray(self.camera_matrix, dtype=np.float64)
        if self.depth_intrinsics is None:
            object.__setattr__(self, "depth_intrinsics", self.intrinsics)
        if self.depth_timestamp_ns <= 0:
            object.__setattr__(self, "depth_timestamp_ns", self.timestamp_ns)
        if self.rotation_camera_to_world is None:
            object.__setattr__(self, "rotation_camera_to_world", matrix[:3, :3].copy())
        if self.camera_position_world is None:
            object.__setattr__(
                self,
                "camera_position_world",
                (float(matrix[0, 3]), float(matrix[1, 3]), -float(matrix[2, 3])),
            )