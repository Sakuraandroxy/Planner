"""World-model scoring interfaces."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List


@dataclass
class WorldModelResult:
    """World model scoring result."""

    best_index: int = 0
    scores: List[float] = field(default_factory=list)
    reasoning: str = ""


class BaseWorldModel(ABC):
    """World model interface."""

    @abstractmethod
    def score(self, front_img_b64: str, down_img_b64: str, instruction: str, candidates: List[dict]) -> WorldModelResult:
        ...
