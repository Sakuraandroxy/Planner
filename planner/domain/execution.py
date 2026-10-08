"""Task-owned deadlines and measured execution receipts; clocks are monotonic."""
from dataclasses import dataclass
import math

from .pose import WorldPose


@dataclass(frozen=True)
class ExecutionBudget:
    deadline_monotonic: float

    def __post_init__(self):
        if not math.isfinite(self.deadline_monotonic):
            raise ValueError("deadline must be finite")

    def remaining(self, now: float) -> float:
        return max(0.0, self.deadline_monotonic - now)


@dataclass(frozen=True)
class ExecutionReceipt:
    completed_segments: int
    final_pose: WorldPose
    elapsed_s: float
    # Segment-end measurements, NOT the full continuously sampled flown path.
    measured_chord_length_m: float = 0.0


@dataclass(frozen=True)
class StopResult:
    confirmed: bool
    errors: tuple[str, ...] = ()
