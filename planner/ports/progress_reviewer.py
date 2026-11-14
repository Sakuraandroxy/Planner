from typing import Protocol

from planner.domain.observation import Observation
from planner.domain.progress import ProgressDecision


class ProgressReviewer(Protocol):
    def review(
        self,
        instruction: str,
        initial: Observation,
        current: Observation,
        round_index: int,
    ) -> ProgressDecision: ...
