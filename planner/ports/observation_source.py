from typing import Protocol

from planner.domain.observation import Observation


class ObservationSource(Protocol):
    def capture(self) -> Observation: ...

