from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class StageResult(Generic[T]):
    stage_id: str
    success: bool
    value: T | None = None
    error: str | None = None


@dataclass(frozen=True)
class MissionResult:
    success: bool
    stages: tuple[StageResult[object], ...]
    error: str | None = None

