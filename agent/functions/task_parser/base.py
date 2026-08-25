"""Task parser interfaces and data structures."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class TaskStage:
    """One executable stage parsed from a navigation instruction."""

    index: int
    instruction: str
    mode: str = "target"
    target: str = ""
    relation: str = ""
    action: str = ""
    value: Optional[float] = None
    unit: str = ""
    requires_target: bool = False
    allow_relocalize: bool = False
    completion_condition: str = ""
    ordinal: Optional[int] = None
    selection_rule: str = ""
    stage_kind: str = ""
    # view_relative targets are resolved from the observation made when this
    # stage becomes active, instead of from mission-start/future scans.
    view_relative: bool = False
    # return_target explicitly reuses an instance remembered by an earlier
    # stage and therefore always takes precedence over view_relative.
    return_target: bool = False
    # Explicit continuity such as "the same building". Otherwise a new entity
    # stage must not silently reuse the immediately preceding physical target.
    same_target: bool = False
    # auxiliary_targets 是当前目标的空间锚点/限定物，例如“灌木丛旁边的红车”里的 bush。
    auxiliary_targets: List[str] = field(default_factory=list)

    @property
    def target_query(self) -> str:
        return self.target if self.mode in ("target", "detect") else ""

    @property
    def is_direct_action(self) -> bool:
        return self.mode == "action" and bool(self.action)


class BaseTaskParser(ABC):
    """Task parser interface."""

    @abstractmethod
    def parse(self, instruction: str) -> List[TaskStage]:
        ...
