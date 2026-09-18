from planner.domain.mission import MissionPlan, NavigationParameters, TaskKind
from planner.errors import ProtocolError


class MissionValidator:
    def __init__(self, supported_kinds: set[TaskKind]):
        self.supported_kinds = supported_kinds

    def validate(self, plan: MissionPlan) -> None:
        if plan.protocol_version != "mission_plan_v1":
            raise ProtocolError(f"unsupported mission protocol: {plan.protocol_version}")
        if not plan.stages:
            raise ProtocolError("mission contains no stages")
        seen: set[str] = set()
        for stage in plan.stages:
            if not stage.stage_id or stage.stage_id in seen:
                raise ProtocolError("mission stage IDs must be non-empty and unique")
            seen.add(stage.stage_id)
            if stage.kind not in self.supported_kinds:
                raise ProtocolError(f"unsupported task kind: {stage.kind.value}")
            if stage.kind is TaskKind.NAVIGATION:
                if not isinstance(stage.parameters, NavigationParameters):
                    raise ProtocolError("navigation stage has invalid parameters")
                if not stage.parameters.instruction.strip():
                    raise ProtocolError("navigation instruction is empty")

