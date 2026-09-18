from typing import Protocol

from planner.domain.mission import MissionStage
from planner.domain.result import StageResult


class Workflow(Protocol):
    def run(self, stage: MissionStage) -> StageResult[object]: ...

