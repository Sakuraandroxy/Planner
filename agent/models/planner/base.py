"""Planner model base classes and trajectory result types."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List


@dataclass
class TrajectoryResult:
    """Unified trajectory planning output."""

    waypoints: List[List[float]] = field(default_factory=list)
    done: bool = False
    reasoning: str = ""
    actions: List[str] = field(default_factory=list)
    candidates: List[dict] = field(default_factory=list)


class BasePlanner(ABC):
    """Planner backend interface."""

    @abstractmethod
    def plan(
        self,
        front_img,
        down_img,
        instruction: str,
        direction: str = "",
        detected_bbox=None,
        depth_meters=None,
        detection=None,
        down_depth_meters=None,
        relation: str = "",
        target: str = "",
    ) -> TrajectoryResult:
        ...

    def should_stop(self, detected_bbox, depth_meters, threshold: float = 8.0) -> bool:
        if detected_bbox is None or depth_meters is None:
            return False
        x1, y1, x2, y2 = detected_bbox
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        h, w = depth_meters.shape
        if 0 <= cy < h and 0 <= cx < w:
            return float(depth_meters[cy, cx]) < threshold
        return False


def compute_per_step_deltas(result: TrajectoryResult, start_yaw_deg: float = 0.0) -> List[List[float]]:
    if result.actions:
        return _actions_to_per_step_deltas(result.actions, start_yaw_deg)
    if result.waypoints:
        return _waypoints_to_per_step_deltas(result.waypoints, start_yaw_deg)
    return []


def _parse_action_value(action_str: str):
    parts = action_str.strip().split()
    if len(parts) >= 2:
        try:
            return parts[0].lower(), float(parts[1])
        except ValueError:
            pass
    return action_str.strip().lower(), 0.0


def _actions_to_per_step_deltas(actions: List[str], start_yaw_deg: float) -> List[List[float]]:
    yaw = start_yaw_deg
    deltas = []
    for action_str in actions:
        name, value = _parse_action_value(action_str)
        rad = math.radians(yaw)
        if name == "forward":
            deltas.append([round(value * math.cos(rad), 3), round(value * math.sin(rad), 3), 0.0, 0.0])
        elif name == "backward":
            deltas.append([round(-value * math.cos(rad), 3), round(-value * math.sin(rad), 3), 0.0, 0.0])
        elif name == "left":
            deltas.append([0.0, 0.0, 0.0, -value])
            yaw -= value
        elif name == "right":
            deltas.append([0.0, 0.0, 0.0, value])
            yaw += value
        elif name == "up":
            deltas.append([0.0, 0.0, -value, 0.0])
        elif name == "down":
            deltas.append([0.0, 0.0, value, 0.0])
        else:
            deltas.append([0.0, 0.0, 0.0, 0.0])
    return deltas


def _waypoints_to_per_step_deltas(waypoints: List[List[float]], start_yaw_deg: float) -> List[List[float]]:
    deltas = []
    yaw = start_yaw_deg
    prev = [0.0, 0.0, 0.0]
    for cur in waypoints:
        if len(cur) < 3:
            continue
        dx_body = float(cur[0]) - prev[0]
        dy_body = float(cur[1]) - prev[1]
        dz_body = float(cur[2]) - prev[2]
        if abs(dx_body) < 1e-6 and abs(dy_body) < 1e-6 and abs(dz_body) < 1e-6:
            continue
        target_heading = math.degrees(math.atan2(dy_body, dx_body))
        dist = math.hypot(dx_body, dy_body)
        deltas.append(
            [
                round(dist * math.cos(math.radians(target_heading)), 3),
                round(dist * math.sin(math.radians(target_heading)), 3),
                round(dz_body, 3),
                round(target_heading - yaw, 3),
            ]
        )
        yaw = target_heading
        prev = [float(cur[0]), float(cur[1]), float(cur[2])]
    return deltas
