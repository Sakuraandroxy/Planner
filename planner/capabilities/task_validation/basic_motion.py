"""Validate task wiring only; numeric actions have no flight admission gates."""
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.mission import MissionStage, TaskKind
from planner.errors import ProtocolError


class BasicMotionStageValidator:
    def validate(self, stage: MissionStage) -> None:
        if stage.kind is not TaskKind.BASIC_MOTION or not isinstance(stage.parameters, BasicMotionParameters):
            raise ProtocolError("basic motion stage has invalid parameters")
