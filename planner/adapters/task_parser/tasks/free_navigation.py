"""Open-ended road-following wire rules and strict decoding for a ground or aerial vehicle."""
from planner.domain.free_navigation import FreeNavigationParameters
from planner.errors import ProtocolError

FREE_NAVIGATION_RULES = """free_navigation: drive along the observed road or lane for an open-ended
instruction that has NO explicit numeric travel distance/angle and NO specific static object
to approach. Examples: keep driving along the road; follow the lane and avoid obstacles;
continue forward along this street.
Parameters: instruction only. Do not invent targets, distances, speeds, coordinates,
directions, or completion conditions.
The controlled car drives until the user stops the task; there is no automatic completion.
free_navigation is unbounded and must be the final stage. Reject the whole mission if any
stage follows it. Use basic_motion for plain numeric moves and visual_goal_navigation for
object approach. A road-following request that also supplies a numeric distance or angle is
unsupported: reject the whole mission instead of dropping the quantity or converting it to
basic_motion.
Example stage: {"stage_id":"stage_3","kind":"free_navigation","parameters":{
"instruction":"Keep driving along the road and avoid obstacles. 沿道路继续行驶，避开障碍物。"}}.
"""

AERIAL_FREE_NAVIGATION_RULES = """free_navigation: fly along the observed road or street at the current
altitude for an open-ended instruction that has NO explicit numeric travel distance/angle
and NO specific static object to approach. Examples: keep flying along the road; follow the
street and avoid obstacles; continue forward along this road.
Parameters: instruction only. Do not invent targets, distances, speeds, coordinates,
directions, altitudes, or completion conditions.
The controlled UAV flies until the user stops the task; there is no automatic completion.
free_navigation is unbounded and must be the final stage. Reject the whole mission if any
stage follows it. Use basic_motion for plain numeric moves and visual_goal_navigation for
object approach. A road-following request that also supplies a numeric distance or angle is
unsupported: reject the whole mission instead of dropping the quantity or converting it to
basic_motion.
Example stage: {"stage_id":"stage_3","kind":"free_navigation","parameters":{
"instruction":"Keep flying along the road and avoid obstacles. 沿道路继续飞行，避开障碍物。"}}.
"""


def decode_free_navigation(raw: dict[str, object]) -> FreeNavigationParameters:
    if set(raw) != {"instruction"}:
        raise ProtocolError("free navigation requires instruction only")
    try:
        return FreeNavigationParameters(raw["instruction"])
    except (ValueError, TypeError) as exc:
        raise ProtocolError(
            f"invalid free navigation parameters: {exc}"
        ) from exc