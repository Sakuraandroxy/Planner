"""Contracts specific to visual-goal navigation, separate from trajectory prompts."""
from dataclasses import dataclass
from typing import Any
import math

from .observation import Observation
from .pose import WorldPose
from .visual_goal import GoalGeometry, GoalObservation, VisualGoalNavigationParameters


@dataclass(frozen=True)
class VisualGoalTriggerRules:
    radius_m: float = 4.0
    rearm_translation_m: float = 0.5
    rearm_yaw_deg: float = 10.0
    maximum_completion_checks: int = 10
    required_conditions: tuple[str, ...] = (
        "目标身份与原始任务一致，通过当前图像与参考图确认同一目标。",
        "本地框中心测距满足半径上限、已配置高度边界及稳定观测确认条件。",
        "原始任务明确要求的其他观察条件已经满足；不得自行添加要求。",
    )

    def __post_init__(self):
        for name in ("radius_m", "rearm_translation_m", "rearm_yaw_deg"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
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

    def __post_init__(self):
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

