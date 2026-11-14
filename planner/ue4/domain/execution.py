"""Task-owned deadlines and measured execution receipts; clocks are monotonic."""
from dataclasses import dataclass
import math

from .pose import WorldPose
from .pose import RelativePoseDelta
from .trajectory import MotionTrajectory, RelativeTrajectory
from typing import Callable


# Measured pose and continuous yaw in the MotionSegment.start angle branch;
# return True to pause. False still records measurements without requesting a stop.
PoseCheckpoint = Callable[[WorldPose, float], bool]


@dataclass(frozen=True)
class ExecutionBudget:
    deadline_monotonic: float

    def __post_init__(self):
        if isinstance(self.deadline_monotonic, bool) or not isinstance(self.deadline_monotonic, (int, float)) or not math.isfinite(self.deadline_monotonic):
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
    paused: bool = False
    remaining: MotionTrajectory = MotionTrajectory(())
    measured_actions: tuple[RelativePoseDelta, ...] = ()


@dataclass(frozen=True)
class StopResult:
    confirmed: bool
    errors: tuple[str, ...] = ()


@dataclass(frozen=True)
class ExecutionRecord:
    """Measured endpoint history, scoped to one executed batch (not pending)."""
    start: WorldPose
    end: WorldPose
    actions: RelativeTrajectory
    paused: bool = False

    @property
    def translation_m(self) -> float:
        return math.fsum(math.hypot(p.dx, p.dy, p.dz) for p in self.actions.points)

    @property
    def absolute_yaw_deg(self) -> float:
        return math.fsum(abs(p.dyaw_deg) for p in self.actions.points)

    @property
    def signed_yaw_deg(self) -> float:
        return math.fsum(p.dyaw_deg for p in self.actions.points)
