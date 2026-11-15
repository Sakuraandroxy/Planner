"""Numeric-action wire rules and strict decoding for a ground or aerial vehicle."""
from planner.domain.basic_motion import BasicMotionParameters
from planner.errors import ProtocolError

BASIC_MOTION_RULES = """basic_motion: execute relative ground translation or yaw rotation with an
explicit action, direction and distance/angle. Examples: drive forward 5m; reverse 2m;
turn left 90deg. Classify by explicit motion quantity; object ordinals are not motion quantities.
An object-relative goal such as 'drive to the first car' is not a numeric relative action.
Parameters: instruction, forward_m, yaw_right_deg, up_m; return all four fields.
Positive forward_m = forward, negative = backward (reverse).
Positive yaw_right_deg = right turn, negative = left turn; meters/degrees only.
The controlled car cannot fly, ascend/descend, strafe sideways, or drive to an absolute
position/altitude: up_m must always be 0 for this vehicle, and a request for a height
change is rejected instead of converted.
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
"instruction":"Turn left 90 degrees. 左转90度。","forward_m":0,"yaw_right_deg":-90,"up_m":0}}.
Examples: 'reverse 2m' => forward_m=-2; 'left turn 90deg' => yaw_right_deg=-90;
'drive forward 5m and turn right 45deg' => forward_m=5, yaw_right_deg=45.
Do not invent distances, directions, targets, or completion conditions.
"""

# The reference Planner uses right_m/up_m for a UAV; this runtime's drone
# workflow translates along its heading, yaws, and climbs or descends, so the
# aerial rules advertise those three and reject only lateral motion.
AERIAL_BASIC_MOTION_RULES = """basic_motion: execute relative flight translation, altitude change or yaw
rotation with an explicit action, direction and distance/angle. Examples: fly forward 5m;
climb 10m; descend 3m; turn left 90deg.
Classify by explicit motion quantity; object ordinals are not motion quantities.
An object-relative goal such as 'fly to the first car' is not a numeric relative action.
Parameters: instruction, forward_m, yaw_right_deg, up_m; return all four fields.
Positive forward_m = forward, negative = backward.
Positive yaw_right_deg = right turn, negative = left turn; meters/degrees only.
Positive up_m = climb, negative = descend, measured from the altitude at the start of
this stage; 'ascend', 'climb', 'go up', 'rise' and 'hover higher' are all up_m > 0.
The controlled UAV flies forward/backward, climbs and descends, and turns with yaw. It
cannot strafe sideways; reject a lateral request instead of converting it to a turn.
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
"instruction":"Fly forward 5 meters. 向前飞行5米。","forward_m":5,"yaw_right_deg":0,"up_m":0}}.
Example stage: {"stage_id":"stage_1","kind":"basic_motion","parameters":{
"instruction":"Climb 10 meters. 上升10米。","forward_m":0,"yaw_right_deg":0,"up_m":10}}.
Examples: 'fly backward 2m' => forward_m=-2; 'left turn 90deg' => yaw_right_deg=-90;
'ascend 10m' => up_m=10; 'descend 3m' => up_m=-3.
Do not invent distances, directions, targets, or completion conditions.
"""


def decode_basic_motion(raw: dict[str, object]) -> BasicMotionParameters:
    allowed = {"instruction", "forward_m", "yaw_right_deg", "up_m"}
    if set(raw) != allowed:
        raise ProtocolError(
            "basic motion requires instruction, forward_m, yaw_right_deg and up_m"
        )
    try:
        return BasicMotionParameters(**raw)
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"invalid basic motion parameters: {exc}") from exc