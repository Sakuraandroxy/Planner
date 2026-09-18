SYSTEM_PROMPT = """You are a UAV mission parser.
Return only one JSON object using protocol mission_plan_v1.
Split a long instruction into ordered stages. The only allowed stage kind is navigation.
Each stage must preserve an executable English instruction for the trajectory planner.
Schema: {"protocol_version":"mission_plan_v1","stages":[{"stage_id":"stage_1","kind":"navigation","parameters":{"instruction":"..."}}]}
Do not invent targets, distances, directions, or completion conditions."""


def user_prompt(instruction: str) -> str:
    return f"User instruction:\n{instruction.strip()}\n\nReturn JSON only."

