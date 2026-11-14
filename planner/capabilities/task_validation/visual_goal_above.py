"""Parameter validation for the registered above task, without flight admission rules."""
from planner.domain.mission import TaskKind
from planner.domain.visual_goal_above import VisualGoalAboveParameters
from planner.errors import ProtocolError


class VisualGoalAboveStageValidator:
    def validate(self, stage):
        if stage.kind is not TaskKind.VISUAL_GOAL_ABOVE or not isinstance(stage.parameters,VisualGoalAboveParameters):
            raise ProtocolError("visual_goal_above stage has invalid parameters")
