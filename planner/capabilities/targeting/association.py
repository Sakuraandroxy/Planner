"""Immutable first reference, bounded coordinate updates and stage-bound views."""
from planner.domain.visual_goal import GoalReference, GoalObservation, GoalGeometry
from planner.domain.observation import Observation
from planner.domain.target_state import TargetPositionState
from planner.domain.visual_goal import GoalVisibility
import math
from dataclasses import replace
from planner.domain.pose import WorldPose
from planner.services.target_coordinates import horizontal_distance, nearest_surface


def accept_position(target_state: TargetPositionState | None, stage_id: str, observation: Observation,
                    goal: GoalObservation, geometry: GoalGeometry, radius_m: float
                    ) -> tuple[TargetPositionState | None, str | None]:
    """Bind an object to fixed initial surfaces; only refine matching surfaces."""
    if target_state is not None:
        if target_state.stage_id != stage_id:
            raise ValueError('target state belongs to another stage')
        if (observation.timestamp_ns <= target_state.timestamp_ns or
                observation.captured_monotonic < target_state.captured_monotonic):
            return target_state, 'stale_target_observation'
    if (goal.status is not GoalVisibility.VISIBLE or not goal.same_target or
            not geometry.reliable or geometry.world_point is None):
        return target_state, geometry.reason or 'target_not_identified'
    point = geometry.world_point
    if len(point) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
        return target_state, 'invalid_target_position'
    points = tuple(sample.world_point for sample in geometry.surface_samples if sample.valid and sample.world_point is not None)
    points = tuple(dict.fromkeys(points)) or (point,)
    if target_state is None:
        point = nearest_surface(points, observation.vehicle_pose)
        return TargetPositionState(stage_id, point, point, observation.capture_id,
            observation.captured_monotonic, observation.timestamp_ns,
            position_source=geometry.position_source, anchor_surface_points_world=points,
            surface_points_world=points), None
    anchors = target_state.anchor_surface_points_world or (target_state.anchor_world_point,)
    current = list(target_state.known_surfaces)
    matched = False
    for index, anchor in enumerate(anchors):
        candidate = min(points, key=lambda p: math.dist(p,anchor))
        if math.dist(candidate,anchor) <= radius_m:
            current[index] = candidate
            matched = True
    if not matched:
        return target_state, 'new_surface_not_added_to_fixed_target'
    point = nearest_surface(current, observation.vehicle_pose)
    return replace(target_state, world_point=point, capture_id=observation.capture_id,
        captured_monotonic=observation.captured_monotonic, timestamp_ns=observation.timestamp_ns,
        position_source=geometry.position_source, anchor_surface_points_world=anchors,
        surface_points_world=tuple(current)), None


def bound_target_geometry(target_state: TargetPositionState, pose: WorldPose) -> GoalGeometry:
    point = nearest_surface(target_state.known_surfaces, pose)
    return GoalGeometry(True, 'horizontal distance to nearest stage-bound surface; not current target depth',
        horizontal_distance(point, pose), point, position_source=target_state.position_source,
        surface_points_world=target_state.known_surfaces)


def bound_target_goal(description: str) -> GoalObservation:
    return GoalObservation(GoalVisibility.AMBIGUOUS, None, None, description,
        'locked target reference; current visibility is not confirmed', False, 'stage_target_state')


def make_reference(observation: Observation, goal: GoalObservation, geometry: GoalGeometry) -> GoalReference:
    width, height = observation.rgb.size
    a, b, c, d = goal.bbox
    crop = observation.rgb.crop((int(a*width), int(b*height), max(int(a*width)+1, int(c*width)),
                                max(int(b*height)+1, int(d*height))))
    return GoalReference(goal.description, crop, geometry.world_point, goal.bbox)
