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

    def validate_limits(self, max_distance_m: float, max_yaw_deg: float) -> None:
        if math.hypot(self.forward_m, self.right_m, self.up_m) > max_distance_m:
            raise ValueError("basic motion exceeds maximum step distance")
        if abs(self.yaw_right_deg) > min(max_yaw_deg, 180.0):
            raise ValueError("basic motion exceeds maximum yaw step (at most 180 degrees)")


@dataclass(frozen=True)
class ConfirmedFreeSpace:
    """Operator-certified STATIC world NED box, valid only for this runtime/scene.

    This is an experiment precondition, not a depth-derived obstacle map.
    Radius includes the body envelope and tracking/position-error allowance.
    """
    confirmed: bool = False
    minimum_world_ned_m: tuple[float, float, float] | None = None
    maximum_world_ned_m: tuple[float, float, float] | None = None
    clearance_radius_m: float = 1.0

    def __post_init__(self) -> None:
        if not isinstance(self.confirmed, bool):
            raise ValueError("free-space confirmed must be a boolean")
        if (isinstance(self.clearance_radius_m, bool) or
                not isinstance(self.clearance_radius_m, (int, float)) or
                not math.isfinite(self.clearance_radius_m) or self.clearance_radius_m <= 0):
            raise ValueError("clearance radius must be finite and positive")
        lo, hi = self.minimum_world_ned_m, self.maximum_world_ned_m
        if lo is None and hi is None and not self.confirmed:
            return
        for point in (lo, hi):
            if not isinstance(point, tuple) or len(point) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point
            ):
                raise ValueError("confirmed space requires two finite world NED triples")
        if any(b-a <= 2*self.clearance_radius_m for a, b in zip(lo, hi)):
            raise ValueError("free-space box must exceed twice the clearance radius on every axis")


@dataclass(frozen=True)
class MotionSafetyDecision:
    allowed: bool
    reason: str


@dataclass(frozen=True)
class BasicMotionCompletionDecision:
    completed: bool
    reason: str
    position_error_m: float
    yaw_error_deg: float


class BasicMotionStatus(str, Enum):
    COMPLETED = "completed"
    BLOCKED = "blocked"
    INVALID = "invalid"
    UNCONFIRMED = "unconfirmed"
    TIMEOUT = "timeout"
    OBSERVATION_ERROR = "observation_error"
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
    position_error_m: float | None = None
    yaw_error_deg: float | None = None
    completion_basis: str = "not_verified"
