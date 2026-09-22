from typing import Protocol

from planner.domain.observation import Observation


class TrajectoryPromptBuilder(Protocol):
    def build(self, instruction: str, observation: Observation, max_points: int) -> str: ...


class ProgressPromptBuilder(Protocol):
    def build(
        self,
        instruction: str,
        initial: Observation,
        current: Observation,
        round_index: int,
    ) -> str: ...
