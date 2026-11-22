"""Open-ended, target-free road-following workflow."""

from planner.domain import TaskKind

from .continuous import FreeNavigationContinuous
from .policy import FreeNavigationStageValidator
from .workflow import FreeNavigationWorkflow

#: Task kind this package implements.
TASK_KIND = TaskKind.FREE_NAVIGATION
#: Port slots the task cannot run without; the assembly derives flags from them.
REQUIRED_PORTS: tuple[str, ...] = ("vehicle", "observation", "streaming")

__all__ = [
    "REQUIRED_PORTS",
    "TASK_KIND",
    "FreeNavigationContinuous",
    "FreeNavigationStageValidator",
    "FreeNavigationWorkflow",
]
