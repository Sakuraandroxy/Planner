"""
Real-time navigation metrics for UAV-VLN.
基于 3DG-VLN 的 metric.py 简化适配，实时计算并在终端显示。

指标:
  - DIST: 当前距目标距离 (m)
  - SR:   Success Rate
  - OSR:  Oracle Success Rate
  - NE:   Navigation Error
  - SPL:  Success weighted by Path Length
  - TL:   Total trajectory Length
"""
import math
import time
from typing import List, Optional


DEFAULT_SUCCESS_RADIUS = 12.0


class MetricsTracker:
    """实时指标追踪器，每次 step 更新，完成后打印汇总。"""

    def __init__(self, target_position: List[float],
                 success_radius: float = DEFAULT_SUCCESS_RADIUS):
        self.target_position = target_position
        self.success_radius = success_radius
        self.positions: List[List[float]] = []
        self.timestamps: List[float] = []
        self.started = time.time()
        self._ever_in_radius = False

    def record_step(self, position: List[float]):
        self.positions.append(position)
        self.timestamps.append(time.time())
        d = self._dist(position, self.target_position)
        if d <= self.success_radius:
            self._ever_in_radius = True

    @property
    def current_distance(self) -> float:
        if not self.positions:
            return float("inf")
        return self._dist(self.positions[-1], self.target_position)

    @property
    def trajectory_length(self) -> float:
        if len(self.positions) < 2:
            return 0.0
        return sum(self._dist(self.positions[i-1], self.positions[i])
                   for i in range(1, len(self.positions)))

    @property
    def straight_line_distance(self) -> float:
        if not self.positions:
            return 0.0
        return self._dist(self.positions[0], self.target_position)

    @property
    def ne(self) -> Optional[float]:
        if not self.positions:
            return None
        return self._dist(self.positions[-1], self.target_position)

    @property
    def sr(self) -> bool:
        return self.ne is not None and self.ne <= self.success_radius

    @property
    def osr(self) -> bool:
        return self._ever_in_radius

    @property
    def spl(self) -> float:
        if not self.sr:
            return 0.0
        ref = self.straight_line_distance
        if ref <= 0:
            return 0.0
        return ref / max(self.trajectory_length, ref)

    @property
    def elapsed(self) -> float:
        return time.time() - self.started

    @property
    def avg_speed(self) -> float:
        e = self.elapsed
        return self.trajectory_length / e if e > 0 else 0.0

    def print_status(self, step: int, max_steps: int):
        d = self.current_distance
        tl = self.trajectory_length
        status = "🎯" if d <= self.success_radius else "➡️"
        print(
            f"[METRICS Step {step}/{max_steps}] {status} "
            f"DIST={d:.1f}m  TL={tl:.1f}m  "
            f"SPD={self.avg_speed:.1f}m/s  TIME={self.elapsed:.0f}s"
        )

    def print_summary(self):
        print()
        print("=" * 55)
        print("  📊 导航指标汇总")
        print("=" * 55)
        print(f"  NE  (导航误差):     {self.ne:.2f}m" if self.ne is not None else "  NE:  N/A")
        print(f"  SR  (成功率):       {'✅ 成功' if self.sr else '❌ 失败'} "
              f"(<={self.success_radius}m)")
        print(f"  OSR (宽松成功率):   {'✅ 曾经进入' if self.osr else '❌ 从未进入'}")
        print(f"  TL  (路径长度):     {self.trajectory_length:.1f}m")
        print(f"  SL  (直线距离):     {self.straight_line_distance:.1f}m")
        print(f"  SPL (路径效率):     {self.spl*100:.1f}%")
        print(f"  时间:               {self.elapsed:.0f}s")
        print(f"  平均速度:           {self.avg_speed:.1f}m/s")
        print(f"  步数:               {len(self.positions)}")
        print("=" * 55)
        print()

    def summary_dict(self) -> dict:
        return {
            "ne_m": round(self.ne, 2) if self.ne is not None else None,
            "sr": self.sr,
            "osr": self.osr,
            "trajectory_length_m": round(self.trajectory_length, 1),
            "spl_pct": round(self.spl * 100, 1),
            "elapsed_s": round(self.elapsed, 0),
            "avg_speed_ms": round(self.avg_speed, 1),
            "steps": len(self.positions),
        }

    @staticmethod
    def _dist(a: List[float], b: List[float]) -> float:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)
