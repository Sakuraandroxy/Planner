from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PlannerConfig:
    """规划器配置"""
    # VLM API 配置
    base_url: str = "http://localhost:8000/v1"
    model_name: str = "Qwen3.5-27B-Q4_K_M"
    api_key: str = ""
    
    # 动作参数
    horizontal_step: float = 5    # forward步长（米）
    max_forward_step: float = 20.0  # 单次最大前进距离（米）
    vertical_step: float = 5          # up/down步长（米）
    yaw_step_deg: float = 15.0          # left/right转角（度）
    max_tracking_yaw_step_deg: float = 20.0  # 看到目标时单次最大旋转角（度）
    
    # 候选轨迹参数
    candidate_count: int = 5
    max_trajectory_length: int = 10
    min_trajectory_length: int = 1
    action_mode: str = "atomic"
    target_depth_enabled: bool = True
    velocity: float = 0.5
    capture_interval: float = 0.1
    temperature: float = 0.8
    seed: Optional[int] = None
    
    # Delta噪声参数
    noise_sigma_dx: float = 0.005
    noise_sigma_dy: float = 0.002
    noise_sigma_dz: float = 0.003
    noise_sigma_dphi: float = 0.5
