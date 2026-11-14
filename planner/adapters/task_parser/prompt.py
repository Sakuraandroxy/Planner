from planner.domain.mission import TaskKind
from .tasks import TaskCodec, builtin_task_codecs

TASK_PARSER_SYSTEM_PROMPT = """You are a UAV mission parser. Return JSON only, protocol mission_plan_v1.
Split a long instruction into ordered stages; preserve all actions and target conditions.
Only use the allowed kinds listed below. Never fall back to a removed navigation kind.
Classify by the requested outcome, using each allowed task description. Never invent a kind.
Reject unsupported actions, contradictory requirements, or numeric motion missing its magnitude.
Object appearance, direction and ordinal descriptions are valid visual-goal identifiers.
Do not reject an object-approach request merely because it gives no travel distance.
For rejected requests, reject the WHOLE mission:
{"protocol_version":"mission_plan_v1","stages":[],"error":"specific reason / clarification needed"}.
Never return an error together with executable stages, or discard unsupported parts.
Normal envelope: {"protocol_version":"mission_plan_v1","stages":[{"stage_id":"stage_1",
"kind":"<one allowed kind>","parameters":{}}]}. Populate parameters using that kind's rules below.
Stage IDs must be non-empty and unique. Stage instruction must preserve the full intent in English
followed by one space and its equivalent Simplified Chinese translation, without language labels.
Reject any action not covered by the allowed task rules; never silently omit it.
"""

def user_prompt(instruction: str) -> str:
    return (f"User instruction:\n{instruction.strip()}\n\n"
            "For each stage, parameters.instruction must contain the executable English instruction, "
            "a single space, and its equivalent Simplified Chinese translation. Do not add language labels. "
            "Preserve targets, stage order, quantities, units and prohibitions in both languages. "
            "Do not invent distances, target coordinates or completion conditions. Return JSON only.")


def task_parser_system_prompt(supported_kinds: set[TaskKind], *, codecs: tuple[TaskCodec, ...] | None = None) -> str:
    available = {}
    for codec in builtin_task_codecs() if codecs is None else codecs:
        if codec.kind in available:
            raise ValueError("duplicate task parser codec")
        available[codec.kind] = codec
    if any(not isinstance(kind, TaskKind) or kind not in available for kind in supported_kinds):
        raise ValueError("allowed task has no parser codec")
    names = ", ".join(sorted(kind.value for kind in supported_kinds)) or "none"
    return (TASK_PARSER_SYSTEM_PROMPT + f"\nAllowed kinds for this runtime: {names}.\n" +
            "".join(available[kind].prompt_rules for kind in sorted(supported_kinds, key=lambda k: k.value)))
