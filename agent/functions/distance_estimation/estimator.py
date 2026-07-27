"""Geometric target pose and distance estimation for AirSim navigation."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Optional, Sequence


def _point3(value: Sequence[float]) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def _distance3(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


@dataclass(frozen=True)
class TargetPoseEstimate:
    stage_key: tuple
    target_world: list[float]
    observer_world: list[float]
    observer_yaw_deg: float
    measured_distance_m: float
    camera: str
    score: float
    updated_at: float


@dataclass(frozen=True)
class DistanceEstimate:
    stage_key: tuple
    distance_m: float
    target_world: list[float]
    current_world: list[float]
    observation_age_s: float


class GeometricTargetDistanceEstimator:
    """Cache a target world pose and cheaply recompute UAV-to-target distance."""

    def __init__(self, config: Optional[dict] = None, sim_config: Optional[dict] = None):
        config = config or {}
        sim_config = sim_config or {}
        self.enabled = bool(config.get("ENABLED", True))
        self.use_for_completion = bool(config.get("USE_FOR_COMPLETION", True))
        self.trigger_radius_m = float(config.get("TRIGGER_RADIUS_M", 0.0))
        self.min_score = float(config.get("MIN_SCORE", 0.4))
        self.max_depth_m = float(config.get("MAX_DEPTH_M", 200.0))
        self.front_fov_deg = float(config.get("FRONT_FOV_DEG", sim_config.get("FRONT_FOV", 90.0)))
        self.down_fov_deg = float(config.get("DOWN_FOV_DEG", sim_config.get("DOWN_FOV", 90.0)))
        self.front_camera_offset = _point3(config.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0]))
        self.down_camera_offset = _point3(config.get("DOWN_CAMERA_OFFSET", [0.0, 0.0, 0.0]))
        self.depth_is_radial = str(config.get("DEPTH_MODE", "radial")).strip().lower() != "planar"
        self.completion_confirmations = max(1, int(config.get("COMPLETION_CONFIRMATIONS", 2)))
        self._estimate: Optional[TargetPoseEstimate] = None
        self._accepted_updates = 0
        self._far_anchor = False

    @property
    def has_target(self) -> bool:
        return self.enabled and self._estimate is not None

    @property
    def target_pose(self) -> Optional[TargetPoseEstimate]:
        return self._estimate

    def clear(self) -> None:
        self._estimate = None
        self._accepted_updates = 0
        self._far_anchor = False

    def completion_ready(self, stage_key: tuple) -> bool:
        estimate = self._estimate
        if estimate is None or estimate.stage_key != tuple(stage_key):
            return False
        return self._far_anchor or self._accepted_updates >= self.completion_confirmations

    def update_from_detection(
        self,
        *,
        stage_key: tuple,
        detection: Any,
        image: Any,
        observer_world: Sequence[float],
        observer_yaw_deg: float,
    ) -> Optional[TargetPoseEstimate]:
        if not self.enabled or detection is None or not getattr(detection, "visible", False):
            return None
        bbox = list(getattr(detection, "bbox", []) or [])
        depth = getattr(detection, "depth_median", None)
        score = float(getattr(detection, "score", 0.0) or 0.0)
        camera = str(getattr(detection, "camera", "front") or "front").strip().lower()
        if len(bbox) < 4 or depth is None or image is None or not hasattr(image, "size"):
            return None
        depth = float(depth)
        if not math.isfinite(depth) or depth <= 0.0 or depth > self.max_depth_m or score < self.min_score:
            return None

        width, height = float(image.size[0]), float(image.size[1])
        if width <= 1.0 or height <= 1.0:
            return None
        center_x = (float(bbox[0]) + float(bbox[2])) * 0.5
        center_y = (float(bbox[1]) + float(bbox[3])) * 0.5
        fov_deg = self.down_fov_deg if camera == "down" else self.front_fov_deg
        fx = width / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
        fy = fx
        ray_camera = [1.0, (center_x - width * 0.5) / fx, (center_y - height * 0.5) / fy]
        if self.depth_is_radial:
            norm = math.sqrt(sum(value * value for value in ray_camera))
            ray_camera = [value / max(norm, 1e-9) for value in ray_camera]
        point_camera = [value * depth for value in ray_camera]

        if camera == "down":
            # down_center uses Pitch=-90 degrees: optical forward maps to body +Z (NED down).
            point_body = [-point_camera[2], point_camera[1], point_camera[0]]
            camera_offset = self.down_camera_offset
        else:
            point_body = point_camera
            camera_offset = self.front_camera_offset
        point_body = [point_body[i] + camera_offset[i] for i in range(3)]

        yaw = math.radians(float(observer_yaw_deg))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        observer = _point3(observer_world)
        target_world = [
            observer[0] + cos_yaw * point_body[0] - sin_yaw * point_body[1],
            observer[1] + sin_yaw * point_body[0] + cos_yaw * point_body[1],
            observer[2] + point_body[2],
        ]
        stage_key = tuple(stage_key)
        previous = self._estimate if self._estimate and self._estimate.stage_key == stage_key else None

        estimate = TargetPoseEstimate(
            stage_key=stage_key,
            target_world=[round(value, 4) for value in target_world],
            observer_world=observer,
            observer_yaw_deg=float(observer_yaw_deg),
            measured_distance_m=depth,
            camera=camera,
            score=score,
            updated_at=time.perf_counter(),
        )
        self._estimate = estimate
        self._accepted_updates = self._accepted_updates + 1 if previous is not None else 1
        self._far_anchor = self._far_anchor or depth > self.trigger_radius_m
        return estimate

    def estimate_distance(
        self,
        *,
        stage_key: tuple,
        current_world: Sequence[float],
    ) -> Optional[DistanceEstimate]:
        estimate = self._estimate
        if not self.enabled or estimate is None or estimate.stage_key != tuple(stage_key):
            return None
        current = _point3(current_world)
        return DistanceEstimate(
            stage_key=estimate.stage_key,
            distance_m=_distance3(current, estimate.target_world),
            target_world=list(estimate.target_world),
            current_world=current,
            observation_age_s=max(0.0, time.perf_counter() - estimate.updated_at),
        )
