"""Common mission admission; task-specific checks are explicitly injected."""
from typing import Collection, Mapping, Protocol

from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.errors import ProtocolError


class StageValidator(Protocol):
    def validate(self, stage: MissionStage) -> None: ...


class MissionValidator:
    def __init__(
        self,
        validators: Mapping[TaskKind, StageValidator],
        *,
        terminal_kinds: Collection[TaskKind] = (),
    ):
        self._validators = dict(validators)
        if any(
            not isinstance(kind, TaskKind)
            or not callable(getattr(value, "validate", None))
            for kind, value in self._validators.items()
        ):
            raise ValueError(
                "every admitted task requires an explicit stage validator"
            )
        self._terminal_kinds = frozenset(terminal_kinds)
        if any(
            not isinstance(kind, TaskKind) or kind not in self._validators
            for kind in self._terminal_kinds
        ):
            raise ValueError(
                "every terminal task kind requires an explicit stage validator"
            )

    @property
    def supported_kinds(self) -> set[TaskKind]:
        return set(self._validators)

    def validate(self, plan: MissionPlan) -> None:
        if plan.protocol_version != "mission_plan_v1":
            raise ProtocolError(
                f"unsupported mission protocol: {plan.protocol_version}"
            )
        if not plan.stages:
            raise ProtocolError("mission contains no stages")
        seen: set[str] = set()
        last = len(plan.stages) - 1
        for index, stage in enumerate(plan.stages):
            if (
                not isinstance(stage.stage_id, str)
                or not stage.stage_id.strip()
                or stage.stage_id in seen
            ):
                raise ProtocolError(
                    "mission stage IDs must be non-empty and unique"
                )
            seen.add(stage.stage_id)
            validator = self._validators.get(stage.kind)
            if validator is None or not isinstance(stage.kind, TaskKind):
                raise ProtocolError(f"unsupported task kind: {stage.kind}")
            if stage.kind in self._terminal_kinds and index != last:
                raise ProtocolError(
                    f"{stage.kind.value} must be the final stage"
                )
            validator.validate(stage)