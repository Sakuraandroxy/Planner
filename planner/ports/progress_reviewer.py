from typing import Protocol

from planner.domain.observation import Observation
from planner.domain.execution import ExecutionRecord
from planner.domain.progress import ProgressDecision
from planner.domain.trajectory import WorldTrajectory


class ProgressReviewer(Protocol):
    def review(
        self,
        instruction: str,
        initial: Observation,
        current: Observation,
        round_index: int,
        *, candidate: WorldTrajectory | None = None,
        timeout_s: float | None = None,
        execution_history: tuple[ExecutionRecord, ...] = (),
    ) -> ProgressDecision: ...
