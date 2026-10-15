"""Explicit lifecycle for a stage workflow; preparation never commands the vehicle."""
from typing import Protocol

from planner.domain.mission import MissionStage
from planner.domain.result import StageResult


class Workflow(Protocol):
    def prepare(self) -> None: ...
    def run(self, stage: MissionStage) -> StageResult[object]: ...
    def close(self) -> None: ...
