from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ExitStatus(str, Enum):
    CONTINUE = "continue"
    COMPLETED = "completed"
    BLOCKED = "blocked"
    STUCK = "stuck"
    LIMIT_REACHED = "limit_reached"


@dataclass(frozen=True)
class ExitDecision:
    status: ExitStatus
    reason: str

    @property
    def should_stop(self) -> bool:
        return self.status is not ExitStatus.CONTINUE

    @property
    def completed(self) -> bool:
        return self.status is ExitStatus.COMPLETED
