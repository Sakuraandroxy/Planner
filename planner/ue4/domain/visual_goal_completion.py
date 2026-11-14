"""Contracts specific to visual-goal navigation, separate from trajectory prompts."""
from dataclasses import dataclass
from typing import Any
import math

from .observation import Observation
from .pose import WorldPose
from .visual_goal import GoalGeometry, GoalObservation, VisualGoalNavigationParameters
from .target_state import TargetPositionState, TargetObservationEvidence


@dataclass(frozen=True)
class VisualGoalTriggerRules:
    radius_m: float = 4.0
    rearm_translation_m: float = 0.5
    rearm_yaw_deg: float = 10.0
    maximum_completion_checks: int = 10
    required_conditions: tuple[str, ...] = (
        "目标身份与原始任务一致，可延续本阶段建立的目标绑定，结合本阶段位置估计与当前图像判断；若提供参考图，须对照参考图，未提供时不得假设存在参考图。不要求当前画面独立重新识别完整目标，明确矛盾不能忽略。",
        "到本阶段已绑定表面点集合的最小水平距离满足半径上限、已配置高度边界及稳定观测条件；允许目标局部、背面或近距离表面视图，不因外形不完整判未完成。",
        "原始任务明确要求的其他观察条件已经满足；不得自行添加要求。",
    )
    pause_radius_m: float | None = None

    @property
    def effective_pause_radius_m(self) -> float:
        """Omitted pause radius preserves the previous single-radius behavior."""
        return self.radius_m if self.pause_radius_m is None else self.pause_radius_m

    def __post_init__(self):
        for name in ("radius_m", "rearm_translation_m", "rearm_yaw_deg"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.pause_radius_m is not None:
            value = self.pause_radius_m
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("pause_radius_m must be finite and positive")
        if (isinstance(self.maximum_completion_checks, bool) or
                not isinstance(self.maximum_completion_checks, int) or self.maximum_completion_checks < 1):
            raise ValueError("maximum_completion_checks must be a positive integer")
        if (not isinstance(self.required_conditions, tuple) or not self.required_conditions or
                any(not isinstance(item, str) or not item.strip() for item in self.required_conditions)):
            raise ValueError("required_conditions must contain non-empty strings")


@dataclass(frozen=True)
class VisualGoalTargetSnapshot:
    stage_id: str
    position_world: tuple[float, float, float]
    captured_monotonic: float
    stage_bound: bool = False
    surface_points_world: tuple[tuple[float, float, float], ...] = ()

    def __post_init__(self):
        if type(self.stage_bound) is not bool:
            raise ValueError('stage_bound must be boolean')
        if not self.stage_id or len(self.position_world) != 3 or not all(math.isfinite(x) for x in self.position_world):
            raise ValueError("invalid visual goal target snapshot")
        if not math.isfinite(self.captured_monotonic) or self.captured_monotonic <= 0:
            raise ValueError("target snapshot requires a positive observation time")


@dataclass(frozen=True)
class VisualGoalCompletionInput:
    stage_id: str
    parameters: VisualGoalNavigationParameters
    observation: Observation
    initial_pose: WorldPose
    goal: GoalObservation
    geometry: GoalGeometry
    rules: VisualGoalTriggerRules
    uncertainty_margin_m: float
    height_tolerance_m: float | None
    confirmation_count: int
    required_confirmation_count: int
    reference_description: str
    reference_rgb: Any = None
    target_state: TargetPositionState | None = None
    target_observation: TargetObservationEvidence | None = None

