"""Geometric target pose and distance estimation for AirSim navigation."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Optional, Sequence

from agent.functions.memory.geometry import estimate_detection_world


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
    camera_id: str = ""
    capture_id: str = ""
    projection_source: str = "legacy"


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
        self.config = dict(config)
        self.enabled = bool(config.get("ENABLED", True))
        self.use_for_completion = bool(config.get("USE_FOR_COMPLETION", True))
        self.trigger_radius_m = float(config.get("TRIGGER_RADIUS_M", 0.0))
        self.min_score = float(config.get("MIN_SCORE", 0.4))
        self.max_depth_m = float(config.get("MAX_DEPTH_M", 200.0))
        self.max_reliable_depth_m = float(config.get("MAX_RELIABLE_DEPTH_M", self.max_depth_m))
        self.front_fov_deg = float(config.get("FRONT_FOV_DEG", sim_config.get("FRONT_FOV", 90.0)))
        self.down_fov_deg = float(config.get("DOWN_FOV_DEG", sim_config.get("DOWN_FOV", 90.0)))
        self.front_camera_offset = _point3(config.get("FRONT_CAMERA_OFFSET", [1.0, 0.0, 0.0]))
        self.down_camera_offset = _point3(config.get("DOWN_CAMERA_OFFSET", [0.0, 0.0, 0.0]))
        self.sim_config = dict(sim_config)
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
        if (
            not math.isfinite(depth)
            or depth <= 0.0
            or depth > self.max_depth_m
            or depth > self.max_reliable_depth_m
            or score < self.min_score
        ):
            return None

        observer = _point3(observer_world)
        target_world = estimate_detection_world(
            detection,
            image,
            observer,
            observer_yaw_deg,
            memory_config=self.config,
            sim_config=self.sim_config,
            camera_frame=getattr(detection, "camera_frame", None),
        )
        if target_world is None:
            return None
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
            camera_id=str(getattr(detection, "camera_id", "") or ""),
            capture_id=str(getattr(detection, "capture_id", "") or ""),
            projection_source=(
                "unified"
                if str(self.config.get("CAMERA_GEOMETRY_MODE", "shadow")).strip().lower() == "unified"
                and getattr(detection, "camera_frame", None) is not None
                else "legacy_shadow" if getattr(detection, "camera_frame", None) is not None else "legacy"
            ),
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
