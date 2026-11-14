from planner.domain.mission import MissionStage, TaskKind
from planner.errors import ProtocolError
from planner.workflows.base import Workflow

#工作流分发器
class WorkflowRouter:
    def __init__(self, workflows: dict[TaskKind, Workflow]):
        self._workflows = dict(workflows)

    @property
    def supported_kinds(self) -> set[TaskKind]:
        return set(self._workflows)

    def workflow_for(self, stage: MissionStage) -> Workflow:
        try:
            return self._workflows[stage.kind]
        except KeyError as exc:
            raise ProtocolError(f"no workflow registered for {stage.kind.value}") from exc

