from dataclasses import dataclass
import math

from planner.domain.observation import Observation
from planner.domain.progress import ExitDecision, ExitStatus, ProgressDecision, ProgressStatus


@dataclass(frozen=True)
class NavigationExitContext:
    round_index: int
    previous: Observation
    current: Observation
    progress: ProgressDecision


class NavigationExitPolicy:
    """Default navigation policy; other task workflows inject their own policy."""

    def __init__(self, max_rounds: int, max_stalled_rounds: int = 2,
                 min_translation_m: float = 0.2, min_yaw_deg: float = 2.0):
        if max_rounds <= 0 or max_stalled_rounds <= 0:
            raise ValueError("navigation exit limits must be positive")
        self.max_rounds = max_rounds
        self.max_stalled_rounds = max_stalled_rounds
        self.min_translation_m = min_translation_m
        self.min_yaw_deg = min_yaw_deg
        self._stalled_rounds = 0

    def begin(self) -> None:
        self._stalled_rounds = 0

    def evaluate(self, context: NavigationExitContext) -> ExitDecision:
        progress = context.progress
        if progress.status is ProgressStatus.COMPLETE:
            return ExitDecision(ExitStatus.COMPLETED, progress.reason)
        if progress.status is ProgressStatus.BLOCKED:
            return ExitDecision(ExitStatus.BLOCKED, progress.reason)
        a, b = context.previous.vehicle_pose, context.current.vehicle_pose
        distance = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        yaw = abs((b.yaw_deg - a.yaw_deg + 180) % 360 - 180)
        self._stalled_rounds = self._stalled_rounds + 1 if (
            distance < self.min_translation_m and yaw < self.min_yaw_deg
        ) else 0
        if self._stalled_rounds >= self.max_stalled_rounds:
            return ExitDecision(
                ExitStatus.STUCK,
                f"no measurable motion for {self._stalled_rounds} rounds; task remains incomplete",
            )
        if context.round_index >= self.max_rounds:
            return ExitDecision(
                ExitStatus.LIMIT_REACHED,
                f"round limit reached ({self.max_rounds}); task completion not confirmed",
            )
        return ExitDecision(ExitStatus.CONTINUE, progress.reason)
