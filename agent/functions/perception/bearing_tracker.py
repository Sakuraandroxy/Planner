"""Short-lived RGB bearing state for targets without trustworthy depth."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Iterable, Optional


def signed_angle_delta_deg(target_deg: float, reference_deg: float) -> float:
    return (float(target_deg) - float(reference_deg) + 180.0) % 360.0 - 180.0


def bbox_center_angle_deg(bbox, image, horizontal_fov_deg: float) -> Optional[float]:
    values = list(bbox or [])
    if len(values) < 4 or image is None or not hasattr(image, "size"):
        return None
    width = float(image.size[0])
    if width <= 1.0:
        return None
    center_x = (float(values[0]) + float(values[2])) * 0.5
    return (center_x / width - 0.5) * float(horizontal_fov_deg)


def metric_depth_usable(detection: Any, config: dict) -> tuple[bool, str]:
    depth = getattr(detection, "depth_median", None)
    if depth is None:
        return False, "unavailable"
    try:
        depth = float(depth)
    except (TypeError, ValueError):
        return False, "invalid"
    if not math.isfinite(depth) or depth <= 0.0:
        return False, "invalid"
    max_depth = float(config.get("METRIC_LOCK_MAX_DEPTH_M", 120.0))
    if depth > max_depth:
        return False, "too_far"
    valid_ratio = getattr(detection, "depth_valid_ratio", None)
    if valid_ratio is not None and float(valid_ratio) < float(config.get("METRIC_DEPTH_MIN_VALID_RATIO", 0.20)):
        return False, "too_sparse"
    mad = getattr(detection, "depth_mad_m", None)
    max_mad_ratio = float(config.get("METRIC_DEPTH_MAX_MAD_RATIO", 0.12))
    if mad is not None and float(mad) > max(0.5, depth * max_mad_ratio):
        return False, "too_noisy"
    return True, "reliable"


@dataclass(frozen=True)
class TargetBearingObservation:
    stage_key: tuple
    relative_angle_deg: float
    world_bearing_deg: float
    score: float
    bbox: tuple[int, int, int, int]
    observed_at: float
    depth_state: str
    range_hint_m: Optional[float] = None

    def age_s(self) -> float:
        return max(0.0, time.perf_counter() - float(self.observed_at))

    def relative_to_yaw(self, yaw_deg: float) -> float:
        return signed_angle_delta_deg(self.world_bearing_deg, yaw_deg)


class TargetBearingTracker:
    """Keep fresh visual direction without pretending it is a 3-D landmark."""

    def __init__(self, config: Optional[dict] = None, sim_config: Optional[dict] = None):
        self.config = dict(config or {})
        self.sim_config = dict(sim_config or {})
        self._observations: dict[tuple, TargetBearingObservation] = {}
        self._lost_counts: dict[tuple, int] = {}

    def clear(self, stage_key: Optional[tuple] = None) -> None:
        if stage_key is None:
            self._observations.clear()
            self._lost_counts.clear()
            return
        key = tuple(stage_key)
        self._observations.pop(key, None)
        self._lost_counts.pop(key, None)

    def record(
        self,
        *,
        stage_key: tuple,
        detection: Any,
        image: Any,
        observer_yaw_deg: float,
    ) -> Optional[TargetBearingObservation]:
        if not bool(self.config.get("BEARING_ONLY_ENABLED", True)):
            return None
        if detection is None or not getattr(detection, "visible", False):
            return None
        camera = str(getattr(detection, "camera", "front") or "front").lower()
        if camera != "front":
            return None
        score = float(getattr(detection, "score", 0.0) or 0.0)
        if score < float(self.config.get("BEARING_MIN_CONFIDENCE", 0.40)):
            return None
        fov = float(self.config.get("FRONT_FOV_DEG", self.sim_config.get("FRONT_FOV", 90.0)))
        relative = bbox_center_angle_deg(getattr(detection, "bbox", None), image, fov)
        if relative is None:
            return None
        usable_depth, depth_state = metric_depth_usable(detection, self.config)
        depth = getattr(detection, "depth_median", None)
        range_hint = None
        if depth is not None:
            try:
                candidate = float(depth)
                range_hint = candidate if math.isfinite(candidate) and candidate > 0.0 else None
            except (TypeError, ValueError):
                pass
        observation = TargetBearingObservation(
            stage_key=tuple(stage_key),
            relative_angle_deg=float(relative),
            world_bearing_deg=(float(observer_yaw_deg) + float(relative)) % 360.0,
            score=score,
            bbox=tuple(int(v) for v in list(getattr(detection, "bbox", []) or [])[:4]),
            observed_at=time.perf_counter(),
            depth_state="metric" if usable_depth else depth_state,
            range_hint_m=range_hint,
        )
        self._observations[observation.stage_key] = observation
        self._lost_counts[observation.stage_key] = 0
        return observation

    def current(self, stage_key: tuple) -> Optional[TargetBearingObservation]:
        observation = self._observations.get(tuple(stage_key))
        if observation is None:
            return None
        if observation.age_s() > float(self.config.get("BEARING_MAX_AGE_S", 6.0)):
            self._observations.pop(tuple(stage_key), None)
            return None
        return observation

    def mark_lost(self, stage_key: tuple) -> int:
        key = tuple(stage_key)
        count = self._lost_counts.get(key, 0) + 1
        self._lost_counts[key] = count
        if count >= int(self.config.get("BEARING_LOST_CONFIRMATIONS", 2)):
            self._observations.pop(key, None)
        return count


def detection_is_excluded_by_bearing(
    detection: Any,
    image: Any,
    exclusions: Iterable[dict],
    *,
    horizontal_fov_deg: float,
) -> bool:
    angle = bbox_center_angle_deg(getattr(detection, "bbox", None), image, horizontal_fov_deg)
    if angle is None:
        return False
    for exclusion in exclusions or []:
        center = float(exclusion.get("bearing_deg", 0.0) or 0.0)
        tolerance = max(0.0, float(exclusion.get("tolerance_deg", 0.0) or 0.0))
        if abs(signed_angle_delta_deg(angle, center)) <= tolerance:
            return True
    return False
