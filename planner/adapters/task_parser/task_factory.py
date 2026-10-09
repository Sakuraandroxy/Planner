from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.basic_motion import BasicMotionParameters
from planner.adapters.task_parser.response_schema import validate_mission_envelope
from planner.errors import ProtocolError
from planner.domain.visual_goal import ArrivalRelation, VisualGoalNavigationParameters


def _visual_parameters(raw):
    allowed = {"instruction", "target_description", "arrival_relation", "standoff_m", "keep_target_in_view"}
    if set(raw)-allowed:
        raise ProtocolError("unknown visual goal parameters")
    try:
        value = raw.get("standoff_m")
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))):
            raise ValueError("standoff must be a JSON number")
        return VisualGoalNavigationParameters(
            raw.get("instruction"), raw.get("target_description"),
            ArrivalRelation(raw.get("arrival_relation", "near")), value,
            raw.get("keep_target_in_view", True),
        )
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"invalid visual goal parameters: {exc}") from exc


def mission_from_dict(data: dict) -> MissionPlan:
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
        if kind is TaskKind.VISUAL_GOAL_NAVIGATION:
            parsed = _visual_parameters(parameters)
        else:
            allowed = {"instruction", "forward_m", "right_m", "up_m", "yaw_right_deg"}
            if set(parameters) != allowed:
                raise ProtocolError("basic motion requires instruction and all four numeric components")
            try:
                parsed = BasicMotionParameters(**parameters)
                parsed.validate_limits(float("inf"), 180.0)
            except (ValueError, TypeError) as exc:
                raise ProtocolError(f"invalid basic motion parameters: {exc}") from exc
        stages.append(MissionStage(
            stage_id=stage_id,
            kind=kind,
            parameters=parsed,
        ))
    return MissionPlan(str(data.get("protocol_version", "")), tuple(stages))

