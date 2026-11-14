"""Typed open-ended road-following commands and measured results."""
from dataclasses import dataclass
from enum import Enum

from .execution import StopResult
from .pose import WorldPose


@dataclass(frozen=True)
class FreeNavigationParameters:
    """One unbounded, target-free driving stage.

    Only the natural-language instruction is supplied: there is no numeric
    magnitude and no object to approach. The workflow drives until the user
    stops the task.
    """

    instruction: str

    def __post_init__(self) -> None:
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("free navigation instruction must be non-empty")


class FreeNavigationStatus(str, Enum):
    EXECUTION_ERROR = "execution_error"
    MODEL_ERROR = "model_error"
    OBSERVATION_ERROR = "observation_error"
    TIMEOUT = "timeout"
    BLOCKED = "blocked"
    STUCK = "stuck"


@dataclass(frozen=True)
class FreeNavigationResult:
    status: FreeNavigationStatus
    reason: str
    planning_calls: int
    elapsed_s: float
    traveled_m: float
    stop: StopResult
    final_pose: WorldPose | None = None