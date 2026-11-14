"""Validate task wiring only; free navigation has no numeric parameters."""
from planner.domain.free_navigation import FreeNavigationParameters
from planner.domain.mission import MissionStage, TaskKind
from planner.errors import ProtocolError


class FreeNavigationStageValidator:
    def validate(self, stage: MissionStage) -> None:
        if stage.kind is not TaskKind.FREE_NAVIGATION or not isinstance(
            stage.parameters, FreeNavigationParameters
        ):
            raise ProtocolError(
                "free navigation stage has invalid parameters"
            )