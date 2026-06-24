from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional
import math
import random


class AtomicAction(Enum):
    """6种原子动作"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"


@dataclass
class Delta:
    """轨迹终点相对于起点的位移和偏航"""
    dx: float = 0.0    # X方向位移（米）
    dy: float = 0.0    # Y方向位移（米）
    dz: float = 0.0    # Z方向位移（米），正=下降，负=上升
    dphi: float = 0.0  # 偏航变化（度），正=右转
    
    def as_list(self) -> List[float]:
        return [self.dx, self.dy, self.dz, self.dphi]


@dataclass
class Trajectory:
    """一条候选轨迹"""
    actions: List[str] = field(default_factory=list)
    clean_delta: Delta = field(default_factory=Delta)
    noisy_delta: Delta = field(default_factory=Delta)
    name: str = "unnamed"
    scale: float = 1.0
    
    def __len__(self) -> int:
        return len(self.actions)

def parse_action(action_str, config=None, scale=1.0):
    """Parse (action_name, value) from an action string.
    Atomic mode: "forward" -> ("forward", config.horizontal_step * scale)
    Value mode:  "forward 2.2" -> ("forward", 2.2)
    """
    from .config import PlannerConfig
    if config is None:
        config = PlannerConfig()
    if " " in action_str:
        name, val_str = action_str.split(" ", 1)
        val = float(val_str)
        if name == AtomicAction.FORWARD.value:
            val = min(val, config.max_forward_step)
        return name, val
    step_map = {
        "forward": min(config.horizontal_step * scale, config.max_forward_step),
        "backward": config.horizontal_step * scale,
        "up": config.vertical_step * scale,
        "down": config.vertical_step * scale,
        "left": config.yaw_step_deg,
        "right": config.yaw_step_deg,
    }
    return action_str, step_map.get(action_str, 0)



def compute_delta(actions: List[str], config: "PlannerConfig" = None, scale: float = 1.0) -> Delta:
    """
    根据原子动作序列计算终点状态。
    假设起点为 (0, 0, 0)，朝向 0°。
    """
    from .config import PlannerConfig
    if config is None:
        config = PlannerConfig()
    
    x, y, z = 0.0, 0.0, 0.0
    yaw = 0.0
    
    for action in actions:
        name, val = parse_action(action, config, scale)
        if name == AtomicAction.FORWARD.value:
            rad = math.radians(yaw)
            x += val * math.cos(rad)
            y += val * math.sin(rad)
        elif name == AtomicAction.BACKWARD.value:
            rad = math.radians(yaw)
            x -= val * math.cos(rad)
            y -= val * math.sin(rad)
        elif name == AtomicAction.LEFT.value:
            yaw -= val
        elif name == AtomicAction.RIGHT.value:
            yaw += val
        elif name == AtomicAction.UP.value:
            z -= val
        elif name == AtomicAction.DOWN.value:
            z += val
    
    return Delta(dx=x, dy=y, dz=z, dphi=yaw)


def add_noise(delta: Delta, config: "PlannerConfig" = None) -> Delta:
    """给Delta加高斯噪声，模拟传感器误差"""
    from .config import PlannerConfig
    if config is None:
        config = PlannerConfig()
    return Delta(
        dx=delta.dx + random.gauss(0, config.noise_sigma_dx),
        dy=delta.dy + random.gauss(0, config.noise_sigma_dy),
        dz=delta.dz + random.gauss(0, config.noise_sigma_dz),
        dphi=delta.dphi + random.gauss(0, config.noise_sigma_dphi),
    )
