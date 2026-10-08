from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from planner.domain.execution import ExecutionReceipt, StopResult


class PlannerError(RuntimeError):
    """Base error for planner failures."""


class ConfigurationError(PlannerError):
    pass


class ProtocolError(PlannerError):
    pass


class ExecutionError(PlannerError):
    receipt: ExecutionReceipt | None = None
    stop_result: StopResult | None = None


class BudgetExceeded(ExecutionError):
    """Task/motion deadline exhausted; never execute a late response."""


class ObservationError(PlannerError):
    """Capture/stop failure; motion must remain disabled."""

