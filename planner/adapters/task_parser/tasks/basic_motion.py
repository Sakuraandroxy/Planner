"""Numeric-action wire rules and strict decoding."""
from planner.domain.basic_motion import BasicMotionParameters
from planner.errors import ProtocolError

BASIC_MOTION_RULES = """basic_motion: execute relative translation or yaw rotation with an explicit action,
direction and distance/angle. Examples: fly forward 5m; ascend 2m; turn left 90deg.
Classify by explicit motion quantity; object ordinals are not motion quantities.
An object-relative goal such as 'fly to the first car' is not a numeric relative action.
Parameters: instruction, forward_m, right_m, up_m, yaw_right_deg; return all four numeric fields.
Positive forward_m = forward, negative = backward; positive right_m = right, negative = left.
Positive up_m = UP, negative = DOWN. Local code converts up_m to NED dz=-up_m.
Positive yaw_right_deg = right turn, negative = left turn; meters/degrees only.
Unrequested components must be 0. Extract requested values; never guess a missing magnitude.
Relative translation uses the horizontal UAV heading at the beginning of that stage, not camera pitch.
Do not convert 'ascend TO 12m altitude' into 'ascend BY 12m'; absolute position/altitude is unsupported.
Rotation magnitude must be at most 180 degrees. Reject multi-turn rotations and vague moves.
Road following, orbit, moving-target following, and passage traversal are not basic_motion.
Use another allowed kind only if its rules explicitly support the action; otherwise reject the whole mission.
Sequential actions ('move then turn') require separate stages. Explicit simultaneous actions may share
one basic_motion stage (translation vector and yaw change completed together).
Example stage: {"stage_id":"stage_1","kind":"basic_motion","parameters":{
"instruction":"Ascend 12 meters. 上升12米。","forward_m":0,"right_m":0,"up_m":12,"yaw_right_deg":0}}.
Examples: 'ascend 12m' => up_m=12; 'left turn 90deg' => yaw_right_deg=-90;
'fly right 5m' => right_m=5. Do not invent distances, directions, targets, or completion conditions.
"""

def decode_basic_motion(raw: dict[str, object]) -> BasicMotionParameters:
    allowed = {"instruction", "forward_m", "right_m", "up_m", "yaw_right_deg"}
    if set(raw) != allowed:
        raise ProtocolError("basic motion requires instruction and all four numeric components")
    try:
        parsed = BasicMotionParameters(**raw)
        parsed.validate_limits(float("inf"), 180.0)
        return parsed
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"invalid basic motion parameters: {exc}") from exc
