"""Wire rules for reaching the top of a static visual target."""
from planner.domain.visual_goal_above import VisualGoalAboveParameters
from planner.errors import ProtocolError

VISUAL_GOAL_ABOVE_RULES = """visual_goal_above: fly above the top of a specified STATIC object or building
and hover. Above means world-vertical, not the upper part of an image.
Examples: fly above the building ahead; fly 5 meters above the red car.
Parameters: instruction (English followed by equivalent Simplified Chinese),
target_description (English, preserve category, appearance, direction and ordinal),
height_above_m (explicit meters above the object's TOP, otherwise null).
When height is null the workflow supplies a minimum top clearance; it may observe from higher.
Never invent the object's height, world coordinates, or a default separation.
Do not classify above as near or as a numeric relative ascent.
Historical target recall, landing, entering a building and moving-target following are not this task.
Example: {"stage_id":"stage_1","kind":"visual_goal_above","parameters":{
"instruction":"Fly 5 meters above the building ahead. 飞到前方建筑上方5米。",
"target_description":"the building ahead","height_above_m":5.0}}.
"""


def decode_visual_goal_above(raw: dict[str, object]) -> VisualGoalAboveParameters:
    if set(raw) - {"instruction", "target_description", "height_above_m"}:
        raise ProtocolError("unknown visual_goal_above parameters")
    try:
        return VisualGoalAboveParameters(raw.get("instruction"), raw.get("target_description"), raw.get("height_above_m"))
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"invalid visual_goal_above parameters: {exc}") from exc
