"""Task parser interfaces and data structures."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
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
