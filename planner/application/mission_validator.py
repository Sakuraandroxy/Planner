"""Common mission admission; task-specific checks are explicitly injected."""
from typing import Mapping, Protocol

from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.errors import ProtocolError


class StageValidator(Protocol):
    def validate(self, stage: MissionStage) -> None: ...


class MissionValidator:
    def __init__(self, validators: Mapping[TaskKind, StageValidator]):
        self._validators = dict(validators)
        if any(not isinstance(kind, TaskKind) or not callable(getattr(value, "validate", None))
               for kind, value in self._validators.items()):
            raise ValueError("every admitted task requires an explicit stage validator")

    @property
    def supported_kinds(self) -> set[TaskKind]:
        return set(self._validators)

    def validate(self, plan: MissionPlan) -> None:
        if plan.protocol_version != "mission_plan_v1":
            raise ProtocolError(f"unsupported mission protocol: {plan.protocol_version}")
        if not plan.stages:
            raise ProtocolError("mission contains no stages")
        seen: set[str] = set()
        for stage in plan.stages:
            if not isinstance(stage.stage_id, str) or not stage.stage_id.strip() or stage.stage_id in seen:
                raise ProtocolError("mission stage IDs must be non-empty and unique")
            seen.add(stage.stage_id)
            validator = self._validators.get(stage.kind)
            if validator is None or not isinstance(stage.kind, TaskKind):
                raise ProtocolError(f"unsupported task kind: {stage.kind}")
            validator.validate(stage)
