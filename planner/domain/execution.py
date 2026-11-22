"""Task-owned deadlines and measured execution receipts; clocks are monotonic."""

import math
from collections.abc import Callable
from dataclasses import dataclass

from .pose import RelativePoseDelta, WorldPose
from .trajectory import MotionTrajectory, RelativeTrajectory

# Measured pose and continuous yaw in the MotionSegment.start angle branch;
# return True to pause. False still records measurements without requesting a stop.
PoseCheckpoint = Callable[[WorldPose, float], bool]


@dataclass(frozen=True)
class ExecutionBudget:
    deadline_monotonic: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.deadline_monotonic, bool)
            or not isinstance(self.deadline_monotonic, (int, float))
            or not math.isfinite(self.deadline_monotonic)
        ):
            raise ValueError("deadline must be finite")

    def remaining(self, now: float) -> float:
        return max(0.0, self.deadline_monotonic - now)


@dataclass(frozen=True)
class ExecutionReceipt:
    completed_segments: int
    final_pose: WorldPose
    elapsed_s: float
    # Segment-end measurements, NOT the full continuously sampled path.
    measured_chord_length_m: float = 0.0
    paused: bool = False
    remaining: MotionTrajectory = MotionTrajectory(())
    measured_actions: tuple[RelativePoseDelta, ...] = ()


@dataclass(frozen=True)
class StopResult:
    confirmed: bool
    errors: tuple[str, ...] = ()


@dataclass(frozen=True)
class CollisionContact:
    """一次已捕获的接触：撞到了什么，适配层是什么时候读到的。

    ``at_monotonic_s`` 填适配层读到事件那一刻的 ``time.monotonic()``：仿真时间戳与
    本地单调钟之间没有已标定的映射，字段名叫 monotonic 就必须是单调钟。
    """

    counterpart: str
    at_monotonic_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.counterpart, str) or not self.counterpart:
            raise ValueError("collision counterpart must be a non-empty string")
        if (
            isinstance(self.at_monotonic_s, bool)
            or not isinstance(self.at_monotonic_s, (int, float))
            or not math.isfinite(self.at_monotonic_s)
        ):
            raise ValueError("collision at_monotonic_s must be finite")


@dataclass(frozen=True)
class CollisionState:
    """一次碰撞端口读取的结果；读取会消费已捕获的事件。"""

    in_contact: bool
    last_contact: CollisionContact | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.in_contact, bool):
            raise ValueError("collision in_contact must be a boolean")
        if self.last_contact is not None and not isinstance(
            self.last_contact, CollisionContact
        ):
            raise ValueError("collision last_contact must be CollisionContact or None")


@dataclass(frozen=True)
class ExecutionRecord:
    """Measured endpoint history, scoped to one executed batch (not pending)."""

    start: WorldPose
    end: WorldPose
    actions: RelativeTrajectory
    paused: bool = False

    @property
    def translation_m(self) -> float:
        return math.fsum(
            math.hypot(p.dx, p.dy, p.dz) for p in self.actions.points
        )

    @property
    def absolute_yaw_deg(self) -> float:
        return math.fsum(abs(p.dyaw_deg) for p in self.actions.points)

    @property
    def signed_yaw_deg(self) -> float:
        return math.fsum(p.dyaw_deg for p in self.actions.points)