"""Pure translation handoff before a stopped endpoint, in world NED meters."""
import math

from planner.domain.pose import WorldPose


def translation_handoff(start: WorldPose, target: WorldPose, speed: float,
                        acceleration: float, jerk: float) -> WorldPose | None:
    """Reserve the distance needed to brake from speed with zero initial acceleration.

    The symmetric jerk-limited profile is triangular at low speed and has an
    acceleration plateau at higher speed. Short moves use endpoint hold alone.
    This is a motion reference, not a clearance or trajectory admission check.
    """
    delta = (target.x-start.x, target.y-start.y, target.z-start.z)
    distance = math.hypot(*delta)
    if speed <= acceleration*acceleration/jerk:
        braking_distance = speed*math.sqrt(speed/jerk)
    else:
        braking_distance = .5*speed*(speed/acceleration+acceleration/jerk)
    if distance <= braking_distance:
        return None
    fraction = (distance-braking_distance)/distance
    return WorldPose(start.x+fraction*delta[0], start.y+fraction*delta[1],
                     start.z+fraction*delta[2], target.yaw_deg)