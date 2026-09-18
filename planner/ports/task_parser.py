from typing import Protocol

from planner.domain.mission import MissionPlan


class TaskParser(Protocol):
    def parse(self, instruction: str) -> MissionPlan: ...

