"""Explicit lifecycle for a stage workflow; preparation never commands the vehicle."""
from typing import Protocol

from planner.domain import MissionStage
from planner.domain import StageResult


class Workflow(Protocol):
    def prepare(self) -> None: ...
    def run(self, stage: MissionStage) -> StageResult[object]: ...
    def close(self) -> None: ...