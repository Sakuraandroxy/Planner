"""Task-independent summaries alongside each task's strongly typed details."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Generic, TypeVar

T = TypeVar("T")


class CompletionBasis(str, Enum):
    NOT_VERIFIED = "not_verified"
    LOCAL_MEASUREMENT = "local_measurement"
    MODEL_WITH_LOCAL_CHECKS = "model_assessment_with_local_checks"


@dataclass(frozen=True)
class ResultMeasurement:
    name: str
    value: float | int | str | bool | None
    unit: str = ""


@dataclass(frozen=True)
class StageSummary:
    status: str
    reason: str
    completion_basis: CompletionBasis = CompletionBasis.NOT_VERIFIED
    completion_confirmed: bool = False
    diagnostics_path: str | None = None
    measurements: tuple[ResultMeasurement, ...] = ()

    def __post_init__(self):
        if not isinstance(self.status, str) or not self.status.strip() or not isinstance(self.reason, str):
            raise ValueError("stage summary requires a status and reason")
        if not isinstance(self.measurements, tuple) or any(not isinstance(item, ResultMeasurement) for item in self.measurements):
            raise ValueError("stage measurements must be an immutable typed tuple")
        if not isinstance(self.completion_basis, CompletionBasis) or type(self.completion_confirmed) is not bool:
            raise ValueError("completion summary requires a typed basis and boolean verdict")
        if self.completion_confirmed and self.completion_basis is CompletionBasis.NOT_VERIFIED:
            raise ValueError("confirmed completion requires an evidence source")


@dataclass(frozen=True)
class StageResult(Generic[T]):
    stage_id: str
    success: bool
    value: T | None = None
    error: str | None = None
    summary: StageSummary | None = None

    def __post_init__(self):
        if type(self.success) is not bool or (self.summary is not None and not isinstance(self.summary, StageSummary)):
            raise ValueError("stage result requires a boolean success and typed summary")
        if not self.success and self.summary is not None and self.summary.completion_confirmed:
            raise ValueError("failed stage cannot claim confirmed completion")


@dataclass(frozen=True)
class MissionResult:
    success: bool
    stages: tuple[StageResult[object], ...]
    error: str | None = None
