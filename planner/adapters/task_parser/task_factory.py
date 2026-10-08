from planner.domain.mission import MissionPlan, MissionStage, NavigationParameters, TaskKind
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
    raw_stages = data.get("stages")
    if not isinstance(raw_stages, list):
        raise ProtocolError("task parser response has no stages list")
    stages = []
    for index, raw in enumerate(raw_stages, start=1):
        if not isinstance(raw, dict):
            raise ProtocolError("mission stage must be an object")
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
            if set(parameters)-{"instruction"} or not isinstance(parameters.get("instruction"), str):
                raise ProtocolError("invalid navigation parameters")
            parsed = NavigationParameters(instruction=parameters["instruction"].strip())
        stages.append(MissionStage(
            stage_id=str(raw.get("stage_id") or f"stage_{index}"),
            kind=kind,
            parameters=parsed,
        ))
    return MissionPlan(str(data.get("protocol_version", "")), tuple(stages))

