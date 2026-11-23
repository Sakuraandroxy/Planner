"""Turn planner objects into the variables a prompt template renders.

Nothing here builds prompt text. The functions read domain objects and return
plain Python values, so wording and whitespace stay in the templates.
"""

from __future__ import annotations

import json

from planner.domain import Observation
from planner.domain import RelativePoseDelta
from planner.domain import RelativeTrajectory
from planner.domain import TargetObservationEvidence
from planner.domain import VisualGoalCompletionInput
from planner.domain import WorldPose
from planner.domain import depth_display_description


def _json(value: object) -> str:
    """Coordinates as JSON text; NaN is an error, Chinese stays readable."""
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


def _flag(value: bool) -> str:
    """Booleans as the lowercase the prompt contract uses."""
    return str(value).lower()


def pose_values(pose: WorldPose) -> dict[str, float]:
    """World pose as ``x``/``y``/``z``/``yaw_deg``."""
    return {"x": pose.x, "y": pose.y, "z": pose.z, "yaw_deg": pose.yaw_deg}


def waypoint_values(point: RelativePoseDelta) -> dict[str, float]:
    """One relative waypoint as ``dx``/``dy``/``dz``/``dyaw_deg``."""
    return {
        "dx": point.dx,
        "dy": point.dy,
        "dz": point.dz,
        "dyaw_deg": point.dyaw_deg,
    }


def trajectory_values(trajectory: RelativeTrajectory) -> dict[str, object]:
    """Waypoints in order, plus how many there are."""
    points = [waypoint_values(point) for point in trajectory.points]
    return {"points": points, "count": len(points)}


def observation_values(observation: Observation) -> dict[str, object]:
    """Capture identity, pose, camera geometry and depth protocol of one observation."""
    return {
        "capture_id": observation.capture_id,
        "frame": observation.frame,
        "pose": pose_values(observation.pose),
        "depth_protocol": depth_display_description(
            observation.depth_mapping, observation.depth_min_m, observation.depth_max_m
        ),
        "camera": {
            "camera_id": observation.camera_id,
            "horizontal_fov_deg": round(observation.intrinsics.horizontal_fov_deg, 3),
            "camera_position_world": [
                round(value, 4) for value in observation.camera_position_world
            ],
            "rotation_camera_to_world": [
                [round(value, 6) for value in row]
                for row in observation.rotation_camera_to_world
            ],
        },
    }


def target_evidence_values(evidence: TargetObservationEvidence) -> dict[str, object]:
    """Detector claim, plus the stage and capture it belongs to."""
    goal = evidence.goal
    return {
        "stage_id": evidence.stage_id,
        "capture_id": evidence.capture_id,
        "captured_monotonic": evidence.captured_monotonic,
        "timestamp_ns": evidence.timestamp_ns,
        "position_update_rejection": evidence.position_update_rejection,
        "goal": {
            "status": goal.status.value,
            "bbox": goal.bbox,
            "description": goal.description,
            "evidence": goal.evidence,
            "same_target": goal.same_target,
            "association_source": goal.association_source,
            "surface_points": goal.surface_points,
        },
    }


def completion_values(request: VisualGoalCompletionInput) -> dict[str, object]:
    """Stage, binding, detector evidence, trigger rules and local measurement."""
    observation = request.observation
    pose = observation.pose
    rules = request.rules
    target_state = request.target_state
    evidence = request.target_observation
    height_tolerance = request.height_tolerance_m
    fresh_evidence = (
        evidence is not None
        and evidence.stage_id == request.stage_id
        and evidence.timestamp_ns <= observation.timestamp_ns
        and evidence.captured_monotonic <= observation.captured_monotonic
    )
    values: dict[str, object] = {
        "stage_id": request.stage_id,
        "instruction": request.parameters.instruction,
        "target_description": request.parameters.target_description,
        "reference_description": request.reference_description,
        "has_reference": request.reference_rgb is not None,
        "depth_protocol": depth_display_description(
            observation.depth_mapping, observation.depth_min_m, observation.depth_max_m
        ),
        "association_source": request.goal.association_source,
        "bound": target_state is not None,
        "current_bbox_json": _json(request.goal.bbox),
        "target_world_point": _json(request.geometry.world_point),
        "position_source": request.geometry.position_source,
        "keep_target_in_view": _flag(request.parameters.keep_target_in_view),
        "initial_pose_json": _json(
            [
                request.initial_pose.x,
                request.initial_pose.y,
                request.initial_pose.z,
                request.initial_pose.yaw_deg,
            ]
        ),
        "has_evidence": fresh_evidence,
        "pause_radius_m": rules.effective_pause_radius_m,
        "radius_m": rules.radius_m,
        "uncertainty_margin_m": request.uncertainty_margin_m,
        "height_tolerance": height_tolerance,
        "pose_json": _json([pose.x, pose.y, pose.z]),
        "yaw_deg": pose.yaw_deg,
        "distance_m": request.geometry.distance_m,
        "radius_met": _flag(
            request.geometry.distance_m + request.uncertainty_margin_m <= rules.radius_m
        ),
        "height_met": _flag(
            height_tolerance is None
            or abs(pose.z - request.initial_pose.z) <= height_tolerance
        ),
        "confirmation_count": request.confirmation_count,
        "required_confirmation_count": request.required_confirmation_count,
        "required_conditions": rules.required_conditions,
    }
    if target_state is not None:
        values.update(
            {
                "anchor_world_point": _json(target_state.anchor_world_point),
                "known_surfaces": _json(target_state.known_surfaces),
                "bound_capture_id": target_state.capture_id,
                "bound_age_s": f"{observation.captured_monotonic - target_state.captured_monotonic:.3f}",
            }
        )
    if evidence is not None and fresh_evidence:
        goal = evidence.goal
        values.update(
            {
                "evidence_capture_id": evidence.capture_id,
                "evidence_age_s": f"{observation.captured_monotonic - evidence.captured_monotonic:.3f}",
                "same_observation": _flag(evidence.capture_id == observation.capture_id),
                "status": goal.status.value,
                "same_target": _flag(goal.same_target),
                "description": goal.description,
                "evidence_association_source": goal.association_source,
                "evidence_text": goal.evidence,
                "evidence_bbox_json": _json(goal.bbox),
                "has_surface_points": goal.surface_points is not None,
                "surface_points_json": _json(goal.surface_points),
                "rejection": evidence.position_update_rejection,
            }
        )
    return values
