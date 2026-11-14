"""Explicit lifecycle for a stage workflow; preparation never commands the vehicle."""
from typing import Protocol

from planner.ue4.domain.mission import MissionStage
from planner.ue4.domain.result import StageResult


class Workflow(Protocol):
    def prepare(self) -> None: ...
    def run(self, stage: MissionStage) -> StageResult[object]: ...
    def close(self) -> None: ...
