"""Deferred explicit assembly: unused workflows construct no optional resources."""
from typing import Callable

from planner.domain.mission import MissionStage
from planner.domain.result import StageResult
from planner.errors import ExecutionError
from .base import Workflow


class ConfiguredWorkflow:
    def __init__(self, factory: Callable[[], Workflow]):
        self.factory = factory
        self.instance: Workflow | None = None
        self._closed = False

    def prepare(self) -> None:
        if self._closed:
            raise ExecutionError("configured workflow is closed")
        if self.instance is None:
            instance = self.factory()
            if any(not callable(getattr(instance, name, None)) for name in ("prepare", "run", "close")):
                close = getattr(instance, "close", None)
                if callable(close):
                    close()
                raise ExecutionError("workflow must implement prepare, run, and close")
            self.instance = instance
        self.instance.prepare()

    def run(self, stage: MissionStage) -> StageResult[object]:
        self.prepare()
        assert self.instance is not None
        return self.instance.run(stage)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.instance is not None:
            self.instance.close()
