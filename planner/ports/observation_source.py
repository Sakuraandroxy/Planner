from typing import Protocol
from concurrent.futures import Future
from planner.domain.execution import ExecutionBudget

from planner.domain.observation import Observation


class ObservationSource(Protocol):
    def capture(self, *, budget: ExecutionBudget | None = None) -> Observation: ...



class ControlledObservationSource(ObservationSource, Protocol):
    """Optional capture request; only the control owner services it."""
    def request_capture(self, timeout_s: float | None = None) -> Future[Observation]: ...
