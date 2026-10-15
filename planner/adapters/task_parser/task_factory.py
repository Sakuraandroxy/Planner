from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.adapters.task_parser.response_schema import validate_mission_envelope
from planner.errors import ProtocolError
from .tasks import TaskCodec, builtin_task_codecs


def mission_from_dict(data: dict, *, codecs: tuple[TaskCodec, ...] | None = None) -> MissionPlan:
    admitted = {}
    for codec in builtin_task_codecs() if codecs is None else codecs:
        if codec.kind in admitted:
            raise ProtocolError("duplicate task codec")
        admitted[codec.kind] = codec
    validate_mission_envelope(data)
    raw_stages = data.get("stages")
    if not isinstance(raw_stages, list):
        raise ProtocolError("task parser response has no stages list")
    stages = []
    seen = set()
    for raw in raw_stages:
        if not isinstance(raw, dict):
            raise ProtocolError("mission stage must be an object")
        if set(raw)-{"stage_id", "kind", "parameters"}:
            raise ProtocolError("unknown mission stage fields")
        stage_id = raw.get("stage_id")
        if not isinstance(stage_id, str) or not stage_id.strip() or stage_id in seen:
            raise ProtocolError("mission stage IDs must be non-empty and unique")
        seen.add(stage_id)
        try:
            kind = TaskKind(raw.get("kind"))
        except ValueError as exc:
            raise ProtocolError("unknown task kind") from exc
        parameters = raw.get("parameters")
        if not isinstance(parameters, dict):
            raise ProtocolError("stage parameters must be an object")
        codec = admitted.get(kind)
        if codec is None:
            raise ProtocolError(f"task kind is not admitted: {kind.value}")
        parsed = codec.decode(parameters)
        stages.append(MissionStage(
            stage_id=stage_id,
            kind=kind,
            parameters=parsed,
        ))
    return MissionPlan(str(data.get("protocol_version", "")), tuple(stages))

