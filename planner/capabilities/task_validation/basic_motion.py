"""Preflight validation for explicit relative motion."""
from dataclasses import dataclass

from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.mission import MissionStage, TaskKind
from planner.errors import ProtocolError


@dataclass(frozen=True)
class BasicMotionStageValidator:
    maximum_step_m: float = 30.0
    maximum_yaw_step_deg: float = 180.0
    free_space_confirmed: bool = True

    def validate(self, stage: MissionStage) -> None:
        if stage.kind is not TaskKind.BASIC_MOTION or not isinstance(stage.parameters, BasicMotionParameters):
            raise ProtocolError("basic motion stage has invalid parameters")
        if not self.free_space_confirmed:
            raise ProtocolError("confirm basic_motion.free_space before executing this mission")
        try:
            stage.parameters.validate_limits(self.maximum_step_m, self.maximum_yaw_step_deg)
        except ValueError as exc:
            raise ProtocolError(str(exc)) from exc
