"""Numeric-action wire rules and strict decoding for a ground or aerial vehicle."""
from planner.domain.basic_motion import BasicMotionParameters
from planner.errors import ProtocolError

BASIC_MOTION_RULES = """basic_motion: execute relative ground translation or yaw rotation with an
explicit action, direction and distance/angle. Examples: drive forward 5m; reverse 2m;
turn left 90deg. Classify by explicit motion quantity; object ordinals are not motion quantities.
An object-relative goal such as 'drive to the first car' is not a numeric relative action.
Parameters: instruction, forward_m, yaw_right_deg; return both numeric fields.
Positive forward_m = forward, negative = backward (reverse).
Positive yaw_right_deg = right turn, negative = left turn; meters/degrees only.
The controlled car cannot fly, ascend/descend, strafe sideways, or drive to an absolute
position/altitude; reject those actions instead of converting them.
Unrequested components must be 0. Extract requested values; never guess a missing magnitude.
Relative translation uses the ground-vehicle heading at the beginning of that stage.
Preserve the requested numeric rotation, including full turns. Reject vague moves.
Road following, orbit, moving-target following, and passage traversal are not basic_motion.
A move that also says to follow the road/lane/street while giving a numeric distance or
angle is not a plain numeric action either: reject the whole mission instead of reducing it
to the number.
Use another allowed kind only if its rules explicitly support the action; otherwise reject the whole mission.
Sequential actions ('move then turn') require separate stages. Explicit simultaneous actions may share
one basic_motion stage (forward translation and yaw change completed together).
Example stage: {"stage_id":"stage_1","kind":"basic_motion","parameters":{
"instruction":"Turn left 90 degrees. 左转90度。","forward_m":0,"yaw_right_deg":-90}}.
Examples: 'reverse 2m' => forward_m=-2; 'left turn 90deg' => yaw_right_deg=-90;
'drive forward 5m and turn right 45deg' => forward_m=5, yaw_right_deg=45.
Do not invent distances, directions, targets, or completion conditions.
"""

# The reference Planner uses right_m/up_m for a UAV; this runtime's drone
# workflow only supports forward translation and yaw, so the aerial rules
# advertise those two fields and reject lateral/vertical actions explicitly.
AERIAL_BASIC_MOTION_RULES = """basic_motion: execute relative flight translation or yaw rotation with an
explicit action, direction and distance/angle. Examples: fly forward 5m; turn left 90deg.
Classify by explicit motion quantity; object ordinals are not motion quantities.
An object-relative goal such as 'fly to the first car' is not a numeric relative action.
Parameters: instruction, forward_m, yaw_right_deg; return both numeric fields.
Positive forward_m = forward, negative = backward (reverse).
Positive yaw_right_deg = right turn, negative = left turn; meters/degrees only.
The controlled UAV flies forward/backward at its current altitude and turns with yaw. It
cannot strafe sideways or change altitude in this runtime; reject those actions instead of
converting them.
Unrequested components must be 0. Extract requested values; never guess a missing magnitude.
Relative translation uses the horizontal UAV heading at the beginning of that stage,
not camera pitch.
Preserve the requested numeric rotation, including full turns. Reject vague moves.
Road following, orbit, moving-target following, and passage traversal are not basic_motion.
A move that also says to follow the road/lane/street while giving a numeric distance or
angle is not a plain numeric action either: reject the whole mission instead of reducing it
to the number.
Use another allowed kind only if its rules explicitly support the action; otherwise reject the whole mission.
Sequential actions ('move then turn') require separate stages. Explicit simultaneous actions may share
one basic_motion stage (forward translation and yaw change completed together).
Example stage: {"stage_id":"stage_1","kind":"basic_motion","parameters":{
"instruction":"Fly forward 5 meters. 向前飞行5米。","forward_m":5,"yaw_right_deg":0}}.
Examples: 'fly backward 2m' => forward_m=-2; 'left turn 90deg' => yaw_right_deg=-90.
Do not invent distances, directions, targets, or completion conditions.
"""


def decode_basic_motion(raw: dict[str, object]) -> BasicMotionParameters:
    allowed = {"instruction", "forward_m", "yaw_right_deg"}
    if set(raw) != allowed:
        raise ProtocolError(
            "basic motion requires instruction, forward_m and yaw_right_deg"
        )
    try:
        return BasicMotionParameters(**raw)
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"invalid basic motion parameters: {exc}") from exc