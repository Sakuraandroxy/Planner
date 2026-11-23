"""Turn planner objects into the variables a prompt template renders.

Nothing here builds prompt text. The functions read domain objects and return
plain Python values, so wording and whitespace stay in the templates.
"""

from __future__ import annotations

import json
import math

from planner.algorithms import point_in_horizontal_body
from planner.algorithms import relative_to_world
from planner.algorithms.depth import FreeSpaceProfile
from planner.domain import Observation
from planner.domain import RelativePoseDelta
from planner.domain import RelativeTrajectory
from planner.domain import TargetDetectionInput
from planner.domain import TargetObservationEvidence
from planner.domain import VisualGoalPlanningContext
from planner.domain import WorldPose
from planner.domain import depth_display_description


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


def detection_values(request: TargetDetectionInput) -> dict[str, str]:
    """Instruction and target name of one detection request."""
    return {
        "instruction": request.instruction,
        "target_description": request.target_description,
    }


def vehicle_values(aerial: bool, /) -> dict[str, object]:
    """Vehicle noun the trajectory rules use, plus the aerial flag."""
    return {"aerial": aerial, "subject": "UAV" if aerial else "vehicle"}


def bearing_label(value: float) -> str:
    """Bearing in degrees, signed, one decimal; zero stays unsigned."""
    return "0.0" if abs(value) < 0.05 else f"{value:+.1f}"


def free_space_profile_values(profile: FreeSpaceProfile) -> dict[str, object]:
    """One measured free-space profile, with every number already formatted."""
    ahead = min(
        profile.sectors, key=lambda sector: abs(sector.bearing_deg), default=None
    )
    window = profile.clear_window_deg
    return {
        "min_height_m": f"{profile.min_height_above_ground_m:.2f}",
        "max_range_m": f"{profile.max_range_m:.1f}",
        "sectors": [_sector_values(sector) for sector in profile.sectors],
        "ahead": None if ahead is None else _sector_values(ahead),
        "window": None
        if window is None
        else {
            "start_deg": bearing_label(window[0]),
            "end_deg": bearing_label(window[1]),
        },
    }


def _sector_values(sector) -> dict[str, object]:
    if not sector.ground_available:
        state = "unknown"
    elif sector.distance_m is None:
        state = "clear"
    else:
        state = "measured"
    return {
        "state": state,
        "label": bearing_label(sector.bearing_deg),
        "distance_m": None if sector.distance_m is None else f"{sector.distance_m:.1f}",
        "forward_m": None if sector.forward_m is None else f"{sector.forward_m:.1f}",
        "right_m": None if sector.right_m is None else f"{sector.right_m:+.1f}",
    }


def visual_goal_values(
    observation: Observation,
    task_context: VisualGoalPlanningContext,
    pending_trajectory: RelativeTrajectory | None,
) -> dict[str, object]:
    """Target geometry of one visual-goal prompt: payload JSON plus the bearing."""
    has_pending = pending_trajectory is not None and bool(pending_trajectory.points)
    tail = (
        relative_to_world(pending_trajectory, observation.pose).poses[-1]
        if has_pending
        else observation.pose
    )
    point = task_context.geometry.world_point
    source_time = task_context.target_captured_monotonic
    surfaces = task_context.geometry.surface_points_world or (point,)
    age = (
        None
        if source_time is None
        else round(observation.captured_monotonic - source_time, 4)
    )
    payload = {
        "target_description": task_context.target_description,
        "distance_mode": "horizontal_to_nearest_known_surface",
        "target_surface_points_relative_to_observation_body_m": [
            point_in_horizontal_body(surface, observation.pose) for surface in surfaces
        ],
        "target_surface_points_relative_to_pending_tail_body_m": [
            point_in_horizontal_body(surface, tail) for surface in surfaces
        ],
        "keep_target_in_view": task_context.keep_target_in_view,
        "target_point_source": (
            task_context.geometry.position_source
            if task_context.goal_observation_is_current
            else "stage_bound_" + task_context.geometry.position_source
        ),
        "target_observation_id": task_context.target_capture_id
        or (observation.capture_id if task_context.goal_observation_is_current else None),
        "planning_observation_id": observation.capture_id,
        "target_age_at_planning_observation_s": age,
        "coordinate_frame": "horizontal_body_forward_right_down",
        "target_relative_to_observation_body_m": point_in_horizontal_body(
            point, observation.pose
        ),
        "target_relative_to_pending_tail_body_m": point_in_horizontal_body(point, tail),
        "new_actions_reference": "pending_tail" if has_pending else "observation",
        "pending_tail_is_predicted": has_pending,
        "goal_observation_is_current": task_context.goal_observation_is_current,
        "arrival_radius_m": task_context.arrival_radius_m,
        "distance_uncertainty_margin_m": task_context.distance_uncertainty_margin_m,
        "maximum_vertical_offset_m": task_context.maximum_vertical_offset_m,
    }
    vector = payload[
        "target_relative_to_pending_tail_body_m"
        if has_pending
        else "target_relative_to_observation_body_m"
    ]
    forward, right, _ = vector
    horizontal = math.hypot(forward, right)
    bearing = (
        f"{math.degrees(math.atan2(right, forward)):+.2f}" if horizontal > 1e-6 else None
    )
    if horizontal <= 1e-6:
        side = "undefined"
    elif right > 0:
        side = "right"
    elif right < 0:
        side = "left"
    else:
        side = "ahead"
    return {
        "payload_json": json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        ),
        "start_frame": (
            "final pending predicted pose" if has_pending else "current observation pose"
        ),
        "has_pending": has_pending,
        "target_vector_json": json.dumps(
            [round(value, 4) for value in vector], separators=(",", ":")
        ),
        "bearing_deg": bearing,
        "side": side,
        "target_age_s": age,
    }
