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
    max_forward_step: float = float("inf") # 单次最大前进距离（米）
    vertical_step: float = 5          # up/down步长（米）
    yaw_step_deg: float = 15.0          # left/right转角（度）
    max_tracking_yaw_step_deg: float = float("inf") # 看到目标时单次最大旋转角（度）
    
    # 候选轨迹参数
    candidate_count: int = 5
    max_trajectory_length: int = 5
    min_trajectory_length: int = 1
    action_mode: str = "atomic"
    target_depth_enabled: bool = True
    scene_obstacle_planning_enabled: bool = True
    velocity: float = 0.5
    capture_interval: float = 0.1
    temperature: float = 0.8
    planner_max_tokens: int = 8192
    task_parser_max_tokens: int = 2048
    thinking_mode: str = "disabled"
    reasoning_effort: str = "default"
    enable_thinking: str = "default"
    seed: Optional[int] = None
    task_manager_enabled: bool = True

    # 上下文管理参数
    context_enabled: bool = False
    context_max_steps: int = 5
    context_camera_fov_deg: float = 90.0
    context_world_pos_update_alpha: float = 0.35

    # 到达半径：由 planner prompt 交给 VLM 判断，不属于上下文状态机
    context_arrival_depth: float = 5.0
    approach_stop_margin: float = 1.0

    # 目标实例锁定：第一次发现任务目标后，禁止后续同类远处目标顶替
    target_identity_enabled: bool = True
    target_identity_world_tolerance_abs: float = 15.0
    target_identity_world_tolerance_ratio: float = 0.35
    target_identity_arrival_radius: float = 6.0
    target_identity_update_alpha: float = 0.35

    # 目标丢失后的独立四向环视重定位
    relocalizer_enabled: bool = True
    relocalizer_view_count: int = 4
    relocalizer_yaw_step_deg: float = 90.0
    relocalizer_settle_seconds: float = 0.3
    relocalizer_confidence_threshold: float = 0.5
    relocalizer_max_tokens: int = 1024
    
    # Delta噪声参数
    noise_sigma_dx: float = 0.005
    noise_sigma_dy: float = 0.002
    noise_sigma_dz: float = 0.003
    noise_sigma_dphi: float = 0.5

