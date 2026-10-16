"""Typed relative commands and measured results; no model/device dependencies."""
from dataclasses import dataclass
from enum import Enum
import math

from .pose import WorldPose
from .execution import StopResult


@dataclass(frozen=True)
class BasicMotionParameters:
    instruction: str
    forward_m: float = 0.0
    right_m: float = 0.0
    up_m: float = 0.0
    yaw_right_deg: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("basic motion instruction must be non-empty")
        values = (self.forward_m, self.right_m, self.up_m, self.yaw_right_deg)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError("basic motion values must be finite numbers, not booleans")
        if not any(v != 0 for v in values):
            raise ValueError("basic motion must contain a non-zero action")


class BasicMotionStatus(str, Enum):
    COMPLETED = "completed"
    EXECUTION_ERROR = "execution_error"


@dataclass(frozen=True)
class BasicMotionResult:
    status: BasicMotionStatus
    reason: str
    start: WorldPose | None
    target: WorldPose | None
    actual: WorldPose | None
    elapsed_s: float
    stop: StopResult
    completion_basis: str = "not_verified"
