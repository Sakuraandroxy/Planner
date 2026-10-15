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


class MotionPaused(Exception):
    """Internal normal-control signal; no task policy belongs in the tracker."""

    def __init__(self, pose, unwrapped_yaw_deg):
        super().__init__("execution pause requested")
        self.pose = pose
        self.unwrapped_yaw_deg = unwrapped_yaw_deg


class ModelCallTimeout(ExecutionError):
    """Control-side model deadline expired; a detached worker may still be running."""



class ResourceCleanupError(PlannerError):
    """Every resource was attempted; original exceptions remain available for diagnostics."""
    def __init__(self, failures: tuple[BaseException, ...]):
        self.failures = failures
        super().__init__(f"{len(failures)} resource cleanup(s) failed: " +
                         ", ".join(type(exc).__name__ for exc in failures))
