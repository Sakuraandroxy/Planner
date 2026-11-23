"""Turn planner objects into the variables a prompt template renders.

Nothing here builds prompt text. The functions read domain objects and return
plain Python values, so wording and whitespace stay in the templates.
"""

from __future__ import annotations

from planner.domain import Observation
from planner.domain import RelativePoseDelta
from planner.domain import RelativeTrajectory
from planner.domain import TargetObservationEvidence
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
