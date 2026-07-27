"""Distance-only stage completion, independent from target-pose estimation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class DistanceArrivalResult:
    checked: bool
    done: bool
    distance_m: Optional[float]
    arrival_radius_m: float
    reason: str


class DistanceArrivalCompletion:
    """Decide completion solely from estimated UAV-to-target distance."""

    name = "distance_arrival"

    def __init__(self, arrival_radius_m: float, enabled: bool = True):
        self.enabled = bool(enabled)
        self.arrival_radius_m = max(0.0, float(arrival_radius_m))

    def evaluate(self, distance_m: Optional[float]) -> DistanceArrivalResult:
        if not self.enabled:
            return DistanceArrivalResult(False, False, distance_m, self.arrival_radius_m, "disabled")
        if distance_m is None:
            return DistanceArrivalResult(False, False, None, self.arrival_radius_m, "target distance unavailable")
        distance = float(distance_m)
        if not math.isfinite(distance) or distance < 0.0:
            return DistanceArrivalResult(False, False, distance, self.arrival_radius_m, "invalid target distance")
        done = distance <= self.arrival_radius_m
        relation = "<=" if done else ">"
        return DistanceArrivalResult(
            True,
            done,
            distance,
            self.arrival_radius_m,
            f"estimated_distance={distance:.2f}m {relation} arrival_radius={self.arrival_radius_m:.2f}m",
        )


def build_distance_arrival_completion(config: dict) -> DistanceArrivalCompletion:
    functions = config.get("FUNCTIONS", {}) or {}
    completion_cfg = functions.get("COMPLETION", {}) or {}
    distance_cfg = functions.get("DISTANCE_ESTIMATION", {}) or {}
    radius = completion_cfg.get("ARRIVAL_RADIUS_M", distance_cfg.get("TRIGGER_RADIUS_M", 8.0))
    enabled = completion_cfg.get("DISTANCE_COMPLETION_ENABLED", True)
    return DistanceArrivalCompletion(radius, enabled=enabled)
