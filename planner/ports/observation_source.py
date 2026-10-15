from typing import Protocol
from concurrent.futures import Future
from planner.domain.execution import ExecutionBudget

from planner.domain.observation import Observation


class ObservationSource(Protocol):
    def capture(self, *, budget: ExecutionBudget | None = None) -> Observation: ...



class ControlledObservationSource(ObservationSource, Protocol):
    """Optional capture request; only the control owner services it."""
    def request_capture(self, timeout_s: float | None = None) -> Future[Observation]: ...


class MovingObservationSource(Protocol):
    """Read-only frame-time observations; no hover or control operations."""
    def capture_moving(self, *, budget: ExecutionBudget | None = None) -> Observation: ...
    def close(self) -> None: ...
