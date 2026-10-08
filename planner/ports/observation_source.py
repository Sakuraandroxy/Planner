from typing import Protocol
from planner.domain.execution import ExecutionBudget

from planner.domain.observation import Observation


class ObservationSource(Protocol):
    def capture(self, *, budget: ExecutionBudget | None = None) -> Observation: ...

