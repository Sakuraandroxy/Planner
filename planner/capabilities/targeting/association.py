import math

from planner.domain.visual_goal import GoalReference, GoalObservation, GoalGeometry
from planner.domain.observation import Observation


def associate(reference: GoalReference | None, goal: GoalObservation, geometry: GoalGeometry, gate_m: float, *,
              matches_reference_region: bool = False) -> tuple[bool, str]:
    if not goal.same_target:
        return False, "VLM cannot establish same target identity"
    if reference is not None:
        if not matches_reference_region:
            return False, "same object does not establish selected region correspondence"
        if geometry.world_point is None or math.dist(reference.world_point, geometry.world_point) > gate_m:
            return False, "selected region position is inconsistent with its fixed world anchor"
    return True, "target reference consistent"


def make_reference(observation: Observation, goal: GoalObservation, geometry: GoalGeometry,
                   region_description: str = "visible surface") -> GoalReference:
    width, height = observation.rgb.size
    a, b, c, d = goal.bbox
    crop = observation.rgb.crop((int(a*width), int(b*height), max(int(a*width)+1, int(c*width)),
                                max(int(b*height)+1, int(d*height))))
    u, v = goal.surface_point
    # A contextual patch within the object box, not a foreground segmentation mask.
    half_w, half_h = (c-a)/4, (d-b)/4
    left, top = int(max(a, u-half_w)*width), int(max(b, v-half_h)*height)
    right = max(left+1, int(min(c, u+half_w)*width))
    bottom = max(top+1, int(min(d, v+half_h)*height))
    region_crop = observation.rgb.crop((left, top, right, bottom))
    region_point = (min(1.0, max(0.0, (u*width-left)/(right-left))),
                    min(1.0, max(0.0, (v*height-top)/(bottom-top))))
    return GoalReference(goal.description, crop, geometry.world_point, goal.bbox,
                         region_crop, region_description, region_point)


def coherent_surface_motion(reference: GoalReference | None, goal: GoalObservation, geometry: GoalGeometry,
                            previous_point: tuple[float, float, float] | None, gate_m: float, uncertainty_m: float, *,
                            matches_reference_region: bool = False) -> bool:
    """Conservative motion evidence, contingent on VLM same-object/surface association.

    One jump or a stationary changed surface is insufficient. Two displaced
    world estimates must continue coherently beyond the calibration margin.
    This remains estimated evidence, not simulator object-pose ground truth.
    """
    if (reference is None or not goal.same_target or not matches_reference_region
            or not geometry.reliable or geometry.world_point is None or previous_point is None):
        return False
    current = geometry.world_point
    if any(math.dist(reference.world_point, p) <= gate_m+uncertainty_m for p in (previous_point, current)):
        return False
    outward = tuple(p-r for p, r in zip(previous_point, reference.world_point))
    displacement = tuple(c-p for c, p in zip(current, previous_point))
    length = math.sqrt(sum(v*v for v in displacement))
    if length <= max(0.5, 2*uncertainty_m):
        return False
    cosine = sum(a*b for a, b in zip(outward, displacement))/(math.dist(previous_point, reference.world_point)*length)
    return cosine > 0.8
