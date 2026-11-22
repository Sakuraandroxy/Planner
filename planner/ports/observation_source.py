"""Observation source port."""

from __future__ import annotations

from concurrent.futures import Future
from typing import Protocol

from planner.domain import ExecutionBudget
from planner.domain import Observation


class ObservationSource(Protocol):
    def capture(self, *, budget: ExecutionBudget | None = None) -> Observation: ...


class ControlledObservationSource(ObservationSource, Protocol):
    """Optional capture request; only the control owner services it."""
    def request_capture(self, timeout_s: float | None = None) -> Future[Observation]: ...


class MovingObservationSource(Protocol):
    """Read-only frame-time observations; no hover or control operations."""
    def capture_moving(self, *, budget: ExecutionBudget | None = None) -> Observation: ...

    def close(self) -> None: ...