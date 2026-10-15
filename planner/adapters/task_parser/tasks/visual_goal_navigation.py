"""Static visual-goal wire rules and strict decoding."""
from planner.domain.visual_goal import ArrivalRelation, VisualGoalNavigationParameters
from planner.errors import ProtocolError

VISUAL_GOAL_RULES = """visual_goal_navigation: identify a specified STATIC object/landmark through a
forward, approximately level camera view and fly near/beside it. Translation may be
forward/backward, left/right or up/down, with yaw turns. Classify by the object-approach goal.
Preserve target category, color, direction, ordinal and requested observations.
Examples: approach the first car to the front-left; approach the red house on the right.
Parameters: instruction (English followed by equivalent Simplified Chinese), target_description (English),
arrival_relation="near", keep_target_in_view=true unless explicitly not requested.
Do not add other parameters. Do not turn above/behind/inside/through, orbit or moving-target
follow into near. Explicit custom target-separation distances are unsupported; reject them
rather than discard the distance or convert it to relative travel.
Example stage: {"stage_id":"stage_2","kind":"visual_goal_navigation","parameters":{
"instruction":"Approach the first car to the front-left. 飞到左前方第一辆车旁。",
"target_description":"the first car to the front-left","arrival_relation":"near","keep_target_in_view":true}}.
"""

def decode_visual_goal(raw: dict[str, object]) -> VisualGoalNavigationParameters:
    allowed = {"instruction", "target_description", "arrival_relation", "keep_target_in_view"}
    if set(raw)-allowed:
        raise ProtocolError("unknown visual goal parameters")
    try:
        return VisualGoalNavigationParameters(
            raw.get("instruction"), raw.get("target_description"),
            ArrivalRelation(raw.get("arrival_relation", "near")),
            raw.get("keep_target_in_view", True),
        )
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"invalid visual goal parameters: {exc}") from exc


