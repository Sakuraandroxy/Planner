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
    max_trajectory_length: int = 5
    min_trajectory_length: int = 1
    action_mode: str = "atomic"
    target_depth_enabled: bool = True
    velocity: float = 0.5
    capture_interval: float = 0.1
    temperature: float = 0.8
    planner_max_tokens: int = 8192
    thinking_mode: str = "disabled"
    reasoning_effort: str = "default"
    enable_thinking: str = "default"
    seed: Optional[int] = None

    # 上下文管理参数
    context_enabled: bool = True
    context_reuse_bbox_enabled: bool = True
    context_local_forward_enabled: bool = True
    context_recovery_enabled: bool = True
    context_max_steps: int = 5
    context_bbox_reuse_max_steps: int = 1
    context_bbox_reuse_max_forward: float = 2.0
    context_depth_jump_ratio: float = 1.5
    context_switch_depth_ratio: float = 0.6
    context_switch_guard_steps: int = 3
    context_max_lost_before_redetect: int = 2
    context_local_forward_min_depth: float = 20.0
    context_local_forward_min_action: float = 2.0
    context_goal_safety_margin: float = 2.0
    context_obstacle_min_safe_depth: float = 4.0
    context_obstacle_safety_margin: float = 2.0
    context_search_yaw_step_deg: float = 15.0
    context_arrival_depth: float = 5.0
    context_front_corridor_half_width: float = 0.03
    context_front_corridor_y_min: float = 0.45
    context_front_corridor_y_max: float = 0.60
    context_target_corridor_half_width: float = 0.05
    context_target_corridor_y_min: float = 0.45
    context_target_corridor_y_max: float = 0.65
    context_camera_fov_deg: float = 90.0
    context_expected_depth_abs_tolerance: float = 5.0
    context_expected_depth_ratio_tolerance: float = 0.25
    context_expected_depth_soft_multiplier: float = 1.8
    context_expected_depth_hard_multiplier: float = 2.5
    context_direction_tolerance_deg: float = 35.0
    context_candidate_anchor_min_score: float = 0.55
    
    # Delta噪声参数
    noise_sigma_dx: float = 0.005
    noise_sigma_dy: float = 0.002
    noise_sigma_dz: float = 0.003
    noise_sigma_dphi: float = 0.5
