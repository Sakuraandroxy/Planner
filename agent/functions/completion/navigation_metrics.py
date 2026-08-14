"""Estimated online navigation metrics for the distance-completion loop."""

from __future__ import annotations

import math
import time
from typing import Optional, Sequence


def _point(value: Sequence[float]) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


class NavigationMetricsTracker:
    """Track TL/SL/NE/SR/OSR/SPL using the cached estimated target pose."""

    def __init__(self, success_radius_m: float):
        self.success_radius_m = float(success_radius_m)
        self.started_at = time.perf_counter()
        self.positions: list[list[float]] = []
        self.trajectory_length_m = 0.0
        self.straight_line_distance_m = 0.0
        self.target_world: Optional[list[float]] = None
        self.target_distance_m: Optional[float] = None
        self.first_target_distance_m: Optional[float] = None
        self.latest_confidence: Optional[float] = None
        self._stage_key = None
        self._stage_start: Optional[list[float]] = None
        self._stage_reference_added = False
        self._stage_reference_distance_m = 0.0
        self._ever_in_radius = False
        self.task_completed = False
        self.plan_steps = 0
        self.summary_printed = False

    @property
    def elapsed_s(self) -> float:
        return max(0.0, time.perf_counter() - self.started_at)

    @property
    def average_speed_mps(self) -> float:
        return self.trajectory_length_m / max(self.elapsed_s, 1e-6)

    @property
    def ne_m(self) -> Optional[float]:
        return self.target_distance_m

    @property
    def sr(self) -> bool:
        return bool(self.task_completed)

    @property
    def osr(self) -> bool:
        return self._ever_in_radius

    @property
    def spl(self) -> float:
        if not self.sr or self.straight_line_distance_m <= 0.0:
            return 0.0
        return self.straight_line_distance_m / max(
            self.trajectory_length_m,
            self.straight_line_distance_m,
        )

    def record_pose(self, position: Sequence[float]) -> None:
        current = _point(position)
        if self.positions:
            delta = _distance(self.positions[-1], current)
            if delta < 0.01:
                return
            self.trajectory_length_m += delta
        self.positions.append(current)
        if self._stage_start is None:
            self._stage_start = current
        if self.target_world is not None:
            self.record_distance(_distance(current, self.target_world))

    def start_stage(self, stage_key) -> None:
        if stage_key == self._stage_key:
            return
        self._stage_key = stage_key
        self._stage_start = list(self.positions[-1]) if self.positions else None
        self._stage_reference_added = False
        self._stage_reference_distance_m = 0.0
        self.target_world = None
        self.target_distance_m = None
        self.first_target_distance_m = None

    def update_target(
        self,
        stage_key,
        target_world,
        confidence: Optional[float] = None,
        *,
        replace_stage_reference: bool = False,
    ) -> None:
        self.start_stage(stage_key)
        self.target_world = _point(target_world)
        self.latest_confidence = None if confidence is None else float(confidence)
        if self._stage_start is not None and (
            not self._stage_reference_added or replace_stage_reference
        ):
            reference_distance = _distance(self._stage_start, self.target_world)
            if self._stage_reference_added:
                self.straight_line_distance_m -= self._stage_reference_distance_m
            self.straight_line_distance_m += reference_distance
            self._stage_reference_distance_m = reference_distance
            self._stage_reference_added = True
        if self.positions:
            self.record_distance(_distance(self.positions[-1], self.target_world))

    def invalidate_target(self, stage_key) -> None:
        if stage_key != self._stage_key:
            return
        self.target_world = None
        self.target_distance_m = None
        self.first_target_distance_m = None
        self.latest_confidence = None

    def record_distance(self, distance_m: float) -> None:
        distance = float(distance_m)
        if not math.isfinite(distance):
            return
        self.target_distance_m = distance
        if self.first_target_distance_m is None:
            self.first_target_distance_m = distance
        if distance <= self.success_radius_m:
            self._ever_in_radius = True

    def print_step(self) -> None:
        self.plan_steps += 1
        ne = "N/A" if self.ne_m is None else f"{self.ne_m:.1f}m"
        confidence = "N/A" if self.latest_confidence is None else f"{self.latest_confidence:.2f}"
        depth_change = "N/A"
        if self.first_target_distance_m is not None and self.target_distance_m is not None:
            depth_change = f"{self.first_target_distance_m:.1f}->{self.target_distance_m:.1f}m"
        print(
            f"  [METRICS Step {self.plan_steps}] dist={self.trajectory_length_m:.1f}m  "
            f"spd={self.average_speed_mps:.1f}m/s  NE={ne}  conf={confidence}  "
            f"target={depth_change}"
        )

    def print_summary(self, *, task_completed: bool = False) -> None:
        if self.summary_printed:
            return
        self.task_completed = bool(self.task_completed or task_completed)
        self.summary_printed = True
        ne = "N/A" if self.ne_m is None else f"{self.ne_m:.2f}m"
        print("\n" + "=" * 55)
        print("  导航全程汇总（目标位置来自锁定目标几何）")
        print("=" * 55)
        print(f"  总规划步数:         {self.plan_steps}")
        print(f"  总耗时:             {self.elapsed_s:.1f}s")
        print(f"  累计飞行距离 (TL):  {self.trajectory_length_m:.1f}m")
        print(f"  直线距离 (SL):      {self.straight_line_distance_m:.1f}m")
        print(f"  平均速度:           {self.average_speed_mps:.1f}m/s")
        print("-" * 55)
        print(f"  论文指标 (success_radius={self.success_radius_m:.1f}m)")
        print(f"  NE  (导航误差):     {ne}")
        print(f"  SR  (成功率):       {'成功' if self.sr else '失败'}")
        print(f"  OSR (宽松成功率):   {'曾经进入' if self.osr else '从未进入'}")
        print(f"  SPL (路径效率):     {self.spl * 100.0:.1f}%")
        print("=" * 55)
