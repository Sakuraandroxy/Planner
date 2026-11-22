from planner.domain import TaskKind

from .policy import BasicMotionStageValidator
from .workflow import BasicMotionWorkflow

#: Task kind this package implements.
TASK_KIND = TaskKind.BASIC_MOTION
#: Port slots the task cannot run without; the assembly derives flags from them.
REQUIRED_PORTS: tuple[str, ...] = ("vehicle",)

__all__ = [
    "REQUIRED_PORTS",
    "TASK_KIND",
    "BasicMotionStageValidator",
    "BasicMotionWorkflow",
]
