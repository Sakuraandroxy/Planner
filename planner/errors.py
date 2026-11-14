"""Typed errors shared by adapters, capabilities and workflows."""

from __future__ import annotations


class ProtocolError(RuntimeError):
    """A model response or request violates the documented protocol."""


class ObservationError(RuntimeError):
    """An observation does not satisfy the geometry contract."""


class PlanningError(RuntimeError):
    """A planner could not produce a usable trajectory."""


class ExecutionError(RuntimeError):
    """A vehicle or execution command could not be completed."""


class SimulatorConnectionError(RuntimeError):
    """The simulator transport is unavailable or was lost."""


class SimulatorNotOwned(RuntimeError):
    """The running simulator stack was not started by the launch script."""


class SimulatorRestartError(RuntimeError):
    """Stopping or relaunching the simulator stack failed."""


class ModelCallTimeout(ExecutionError):
    """A background model call exceeded its request deadline."""


class BudgetExceeded(ExecutionError):
    """The task-level time or request budget is exhausted."""


class ResourceCleanupError(RuntimeError):
    """Every resource was attempted; original exceptions remain available for diagnostics."""

    def __init__(self, failures: tuple[BaseException, ...]):
        self.failures = failures
        super().__init__(
            f"{len(failures)} resource cleanup(s) failed: "
            + ", ".join(type(exc).__name__ for exc in failures)
        )


class MotionPaused(Exception):
    """Internal normal-control signal; no task policy belongs in the tracker."""

    def __init__(self, pose, unwrapped_yaw_deg):
        super().__init__("execution pause requested")
        self.pose = pose
        self.unwrapped_yaw_deg = unwrapped_yaw_deg


class FreeNavigationStalled(Exception):
    """Free navigation made no measurable progress; mapped to the STUCK status."""