"""Typed relative ground commands and measured results; no model/device dependencies."""
import math
from dataclasses import dataclass
from enum import Enum

from .execution import StopResult
from .pose import WorldPose


@dataclass(frozen=True)
class BasicMotionParameters:
    """One target-less relative action: translation, climb and/or yaw.

    Positive ``forward_m`` drives forward, negative reverses. Positive
    ``yaw_right_deg`` turns right, negative turns left. Positive ``up_m`` climbs
    and negative descends, measured from the altitude at the start of the
    stage; the world frame's z points down, so the converter subtracts it.
    Any two or three may be non-zero for one explicit simultaneous action.
    Lateral (strafe) motion is not supported by either vehicle.
    """

    instruction: str
    forward_m: float = 0.0
    yaw_right_deg: float = 0.0
    up_m: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("basic motion instruction must be non-empty")
        values = (self.forward_m, self.yaw_right_deg, self.up_m)
        if any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            for v in values
        ):
            raise ValueError(
                "basic motion values must be finite numbers, not booleans"
            )
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