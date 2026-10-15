"""Target appearance reference and most recent bbox-center world estimate."""
from planner.domain.visual_goal import GoalReference, GoalObservation, GoalGeometry
from planner.domain.observation import Observation


def make_reference(observation: Observation, goal: GoalObservation, geometry: GoalGeometry) -> GoalReference:
    width, height = observation.rgb.size
    a, b, c, d = goal.bbox
    crop = observation.rgb.crop((int(a*width), int(b*height), max(int(a*width)+1, int(c*width)),
                                max(int(b*height)+1, int(d*height))))
    return GoalReference(goal.description, crop, geometry.world_point, goal.bbox)
