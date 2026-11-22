"""Typed errors shared by adapters, capabilities and workflows."""

from __future__ import annotations


class ProtocolError(RuntimeError):
    """A model response or request violates the documented protocol."""


class ObservationError(RuntimeError):
    """An observation does not satisfy the geometry contract."""


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


class CollisionInterrupt(Exception):
    """执行层内部的碰撞信号：``ControlLoop._guard`` 抛出，循环自己接住后退避。

    任务层看不到它。预算耗尽时抛的是 ``ExecutionError``，那才需要任务层处理。
    """

    def __init__(self, contact):
        super().__init__("collision contact detected")
        self.contact = contact


class FreeNavigationStalled(Exception):
    """Free navigation made no measurable progress; mapped to the STUCK status."""