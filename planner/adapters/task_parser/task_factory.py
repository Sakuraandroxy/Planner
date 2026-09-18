from planner.domain.mission import MissionPlan, MissionStage, NavigationParameters, TaskKind
from planner.errors import ProtocolError


def mission_from_dict(data: dict) -> MissionPlan:
    raw_stages = data.get("stages")
    if not isinstance(raw_stages, list):
        raise ProtocolError("task parser response has no stages list")
    stages = []
    for index, raw in enumerate(raw_stages, start=1):
        if not isinstance(raw, dict) or raw.get("kind") != TaskKind.NAVIGATION.value:
            raise ProtocolError("base planner only accepts navigation stages")
        parameters = raw.get("parameters")
        if not isinstance(parameters, dict):
            raise ProtocolError("navigation parameters must be an object")
        instruction = str(parameters.get("instruction", "")).strip()
        stages.append(MissionStage(
            stage_id=str(raw.get("stage_id") or f"stage_{index}"),
            kind=TaskKind.NAVIGATION,
            parameters=NavigationParameters(instruction=instruction),
        ))
    return MissionPlan(str(data.get("protocol_version", "")), tuple(stages))

