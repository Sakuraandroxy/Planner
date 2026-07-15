"""
轨迹规划器抽象基类 + 世界模型输入转换工具。
"""
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class TrajectoryResult:
    """统一轨迹规划输出。"""
    waypoints: List[List[float]] = field(default_factory=list)
    done: bool = False
    reasoning: str = ""
    actions: List[str] = field(default_factory=list)  # 原子动作，如 ["forward 4", "left 30"]
    candidates: List[dict] = field(default_factory=list)  # K条候选，每条 {"actions": [...], "reason": "...", "delta": [...]}


class BasePlanner(ABC):
    """规划器接口。"""

    @abstractmethod
    def plan(self, front_img, down_img, instruction: str,
             direction: str = "", detected_bbox=None,
             depth_meters=None, detection=None,
             down_depth_meters=None,
             relation: str = "", target: str = "") -> TrajectoryResult:
        ...

    def should_stop(self, detected_bbox, depth_meters,
                    threshold: float = 8.0) -> bool:
        if detected_bbox is None or depth_meters is None:
            return False
        import numpy as np
        x1, y1, x2, y2 = detected_bbox
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        h, w = depth_meters.shape
        if 0 <= cy < h and 0 <= cx < w:
            return float(depth_meters[cy, cx]) < threshold
        return False


# ══════════════════════════════════════════════════════════════
#  世界模型输入转换：将任意规划器输出转为逐步 (dx, dy, dz, dφ)
# ══════════════════════════════════════════════════════════════

def compute_per_step_deltas(
    result: TrajectoryResult,
    start_yaw_deg: float = 0.0,
) -> List[List[float]]:
    """将规划器输出转为世界模型可用的逐步 delta 序列。

    返回: [[dx, dy, dz, dphi], ...]
        - dx,dy,dz: 该步在全局坐标系下的位移（米）
        - dphi: 该步的 yaw 变化量（度，正=右转）

    对于原子动作（api_atomic_planner）:
        "forward 4" → [4, 0, 0, 0]    (当前朝向)
        "left 30"   → [0, 0, 0, -30]  (原地旋转)
        "forward 5" → [5, 0, 0, 0]    (新朝向, 全局坐标)

    对于 waypoints（qwen_planner / prompt_planner）:
        [[10,0,0], [20,-5,0], [30,-10,0]]
        → 相邻航点间计算方向差作为 dphi
    """
    if result.actions:
        return _actions_to_per_step_deltas(result.actions, start_yaw_deg)
    if result.waypoints:
        return _waypoints_to_per_step_deltas(result.waypoints, start_yaw_deg)
    return []


def _parse_action_value(action_str: str):
    """解析 "forward 4.5" → ("forward", 4.5)。"""
    parts = action_str.strip().split()
    if len(parts) >= 2:
        try:
            return parts[0].lower(), float(parts[1])
        except ValueError:
            pass
    return action_str.strip().lower(), 0.0


def _actions_to_per_step_deltas(actions: List[str],
                                 start_yaw_deg: float) -> List[List[float]]:
    """原子动作 → 逐步 (dx, dy, dz, dphi), 全局坐标。"""
    yaw = start_yaw_deg
    deltas = []
    for action_str in actions:
        name, value = _parse_action_value(action_str)
        rad = math.radians(yaw)
        if name == "forward":
            deltas.append([
                round(value * math.cos(rad), 3),
                round(value * math.sin(rad), 3),
                0.0, 0.0,
            ])
        elif name == "backward":
            deltas.append([
                round(-value * math.cos(rad), 3),
                round(-value * math.sin(rad), 3),
                0.0, 0.0,
            ])
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


def _waypoints_to_per_step_deltas(waypoints: List[List[float]],
                                   start_yaw_deg: float) -> List[List[float]]:
    """机体坐标 waypoints → 逐步 (dx, dy, dz, dphi), 全局坐标。

    waypoints 都是相对起始位置的机体坐标系偏移。相邻 waypoints
    之间的方向差即为该步需要的 dphi（模拟 ForwardOnly 自动旋转）。
    """
    if len(waypoints) < 2:
        # 单点或无点：起点 → 唯一点
        if waypoints:
            wp = waypoints[0]
            return [[wp[0], wp[1], wp[2], 0.0]]
        return []

    deltas = []
    yaw = start_yaw_deg

    # 第一个 waypoint: 从起点 (0,0,0) → wp[0]
    wp_prev = [0.0, 0.0, 0.0]
    for wp_cur in waypoints:
        if wp_cur[0] == 0 and wp_cur[1] == 0 and wp_cur[2] == 0:
            continue  # 跳过零位移航点

        # 方向：从 wp_prev 指向 wp_cur
        dx_body = wp_cur[0] - wp_prev[0]
        dy_body = wp_cur[1] - wp_prev[1]
        dz_body = wp_cur[2] - wp_prev[2]

        # 需要的目标 yaw（在全局坐标系下的方向）
        target_heading = math.degrees(math.atan2(dy_body, dx_body))
        # yaw 变化 = 目标方向 - 当前方向
        dphi = target_heading - yaw

        # 位移大小
        dist = math.sqrt(dx_body ** 2 + dy_body ** 2)

        deltas.append([
            round(dist * math.cos(math.radians(target_heading)), 3),
            round(dist * math.sin(math.radians(target_heading)), 3),
            round(dz_body, 3),
            round(dphi, 3),
        ])

        yaw = target_heading
        wp_prev = wp_cur

    return deltas
