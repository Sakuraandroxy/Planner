import math

from planner.domain.visual_goal import GoalReference


def associate(reference, goal, geometry, gate_m: float) -> tuple[bool, str]:
    if not goal.same_target:
        return False, "VLM cannot establish same target identity"
    if reference is not None and geometry.world_point is not None and math.dist(
        reference.world_point, geometry.world_point
    ) > gate_m:
        return False, "surface identity/position changed; static-target evidence is unreliable"
    return True, "target reference consistent"


def make_reference(observation, goal, geometry):
    width, height = observation.rgb.size
    a, b, c, d = goal.bbox
    crop = observation.rgb.crop((int(a*width), int(b*height), max(int(a*width)+1, int(c*width)),
                                max(int(b*height)+1, int(d*height))))
    return GoalReference(goal.description, crop, geometry.world_point, goal.bbox)


def coherent_surface_motion(reference, goal, geometry, previous_point, gate_m, uncertainty_m):
    """Conservative motion evidence, contingent on VLM same-object/surface association.

    One jump or a stationary changed surface is insufficient. Two displaced
    world estimates must continue coherently beyond the calibration margin.
    This remains estimated evidence, not simulator object-pose ground truth.
    """
    if reference is None or not goal.same_target or not geometry.reliable or previous_point is None:
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
