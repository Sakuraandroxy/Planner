from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TaskKind(str, Enum):
    NAVIGATION = "navigation"


@dataclass(frozen=True)
class NavigationParameters:
    instruction: str


@dataclass(frozen=True)
class MissionStage:
    stage_id: str
    kind: TaskKind
    parameters: NavigationParameters


@dataclass(frozen=True)
class MissionPlan:
    protocol_version: str
    stages: tuple[MissionStage, ...]

