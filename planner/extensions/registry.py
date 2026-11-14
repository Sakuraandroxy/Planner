"""Small explicit task allow-list, assembled at startup without import side effects."""
from dataclasses import dataclass

from planner.adapters.task_parser.tasks import TaskCodec
from planner.application.mission_validator import StageValidator
from planner.domain.mission import TaskKind
from planner.workflows.base import Workflow


@dataclass(frozen=True)
class TaskRegistration:
    kind: TaskKind
    codec: TaskCodec
    validator: StageValidator
    workflow: Workflow

    def __post_init__(self):
        if not isinstance(self.kind, TaskKind) or not isinstance(self.codec, TaskCodec) or self.codec.kind is not self.kind:
            raise ValueError("task registration requires matching finite task kind and codec")
        if not callable(getattr(self.validator, "validate", None)):
            raise ValueError("task registration requires a parameter validator")
        if any(not callable(getattr(self.workflow, name, None)) for name in ("prepare", "run", "close")):
            raise ValueError("task registration requires the complete workflow lifecycle")


class ExtensionRegistry:
    def __init__(self):
        self._tasks: dict[TaskKind, TaskRegistration] = {}

    def register(self, task: TaskRegistration) -> None:
        if task.kind in self._tasks:
            raise ValueError(f"duplicate task registration: {task.kind.value}")
        self._tasks[task.kind] = task

    @property
    def tasks(self) -> tuple[TaskRegistration, ...]:
        return tuple(self._tasks.values())
