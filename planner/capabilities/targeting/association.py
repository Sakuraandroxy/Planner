"""Immutable first reference, bounded coordinate updates and memory-derived views."""
from planner.domain.visual_goal import GoalReference, GoalObservation, GoalGeometry
from planner.domain.observation import Observation
from planner.domain.target_memory import TargetPositionMemory
from planner.domain.visual_goal import GoalVisibility
import math
from dataclasses import replace
from planner.domain.pose import WorldPose


def accept_position(memory: TargetPositionMemory | None, stage_id: str, observation: Observation,
                    goal: GoalObservation, geometry: GoalGeometry, radius_m: float
                    ) -> tuple[TargetPositionMemory | None, str | None]:
    """Pure acceptance rule; every update is bounded by the FIRST anchor."""
    if memory is not None:
        if memory.stage_id != stage_id:
            raise ValueError('target memory belongs to another stage')
        if (observation.timestamp_ns <= memory.timestamp_ns or
                observation.captured_monotonic < memory.captured_monotonic):
            return memory, 'stale_target_observation'
    if (goal.status is not GoalVisibility.VISIBLE or not goal.same_target or
            not geometry.reliable or geometry.world_point is None):
        return memory, geometry.reason or 'target_not_identified'
    point = geometry.world_point
    if len(point) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
        return memory, 'invalid_target_position'
    if memory is not None and math.dist(memory.anchor_world_point, point) > radius_m:
        return memory, 'outside_initial_target_update_radius'
    if memory is None:
        return TargetPositionMemory(stage_id, point, point, observation.capture_id,
            observation.captured_monotonic, observation.timestamp_ns), None
    return replace(memory, world_point=point, capture_id=observation.capture_id,
        captured_monotonic=observation.captured_monotonic, timestamp_ns=observation.timestamp_ns), None


def remembered_geometry(memory: TargetPositionMemory, pose: WorldPose) -> GoalGeometry:
    return GoalGeometry(True, 'distance to remembered bbox-center depth estimate; not current target depth',
        math.dist((pose.x, pose.y, pose.z), memory.world_point), memory.world_point)


def remembered_goal(description: str) -> GoalObservation:
    return GoalObservation(GoalVisibility.AMBIGUOUS, None, None, description,
        'locked target reference; current visibility is not confirmed', False, 'stage_target_memory')


def make_reference(observation: Observation, goal: GoalObservation, geometry: GoalGeometry) -> GoalReference:
    width, height = observation.rgb.size
    a, b, c, d = goal.bbox
    crop = observation.rgb.crop((int(a*width), int(b*height), max(int(a*width)+1, int(c*width)),
                                max(int(b*height)+1, int(d*height))))
    return GoalReference(goal.description, crop, geometry.world_point, goal.bbox)
