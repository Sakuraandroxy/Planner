"""Turn planner objects into the variables a prompt template renders.

Nothing here builds prompt text. The functions read domain objects and return
plain Python values, so wording and whitespace stay in the templates.
"""

from __future__ import annotations

import json

from planner.algorithms import world_pose_delta_in_reference_frame
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


def camera_values(observation: Observation) -> dict[str, object]:
    """Camera id, field of view and extrinsics, rounded as the prompt shows them."""
    return {
        "camera_id": observation.camera_id,
        "horizontal_fov_deg": round(observation.intrinsics.horizontal_fov_deg, 3),
        "camera_position_world": [
            round(value, 4) for value in observation.camera_position_world
        ],
        "rotation_camera_to_world": [
            [round(value, 6) for value in row]
            for row in observation.rotation_camera_to_world
        ],
    }


def observation_values(observation: Observation) -> dict[str, object]:
    """Capture identity, pose, camera geometry and depth protocol of one observation."""
    return {
        "capture_id": observation.capture_id,
        "frame": observation.frame,
        "pose": pose_values(observation.pose),
        "depth_protocol": depth_display_description(
            observation.depth_mapping, observation.depth_min_m, observation.depth_max_m
        ),
        "camera": camera_values(observation),
    }


def compact_json(value: object) -> str:
    """JSON without separator whitespace, as the prompt examples show it."""
    return json.dumps(value, separators=(",", ":"))


def pose_vector_json(pose: WorldPose) -> str:
    """One world pose as compact ``[x,y,z,yaw_deg]``."""
    return compact_json([pose.x, pose.y, pose.z, pose.yaw_deg])


def points_json(trajectory: RelativeTrajectory | None) -> str:
    """Waypoints as compact ``[[dx,dy,dz,dyaw_deg], ...]``; ``None`` renders as empty."""
    points = [] if trajectory is None else [
        [point.dx, point.dy, point.dz, point.dyaw_deg]
        for point in trajectory.points
    ]
    return compact_json(points)


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


def trajectory_refine_values(
    instruction: str,
    observation: Observation,
    *,
    aerial: bool = False,
    current_guidance: str | None = None,
    guidance_start: WorldPose | None = None,
    round_index: int = 1,
    initial_pose: WorldPose | None = None,
    previous_trajectory: RelativeTrajectory | None = None,
    previous_trajectory_start: WorldPose | None = None,
    pending_trajectory: RelativeTrajectory | None = None,
) -> dict[str, object]:
    """Every variable ``trajectory_refine`` reads for one planning round.

    ``guidance_text`` and ``previous_points_json`` are ``None`` when the caller
    supplied nothing, so the template can state the absence itself.
    """
    start_pose = initial_pose or observation.pose
    guidance_text = current_guidance.strip() if current_guidance else ""
    has_history = previous_trajectory is not None
    return {
        "instruction": instruction.strip(),
        "vehicle": "UAV" if aerial else "vehicle",
        "down_label": "NED down" if aerial else "Down",
        "depth_protocol": depth_display_description(
            observation.depth_mapping, observation.depth_min_m, observation.depth_max_m
        ),
        "guidance_text": guidance_text or None,
        "guidance_reference_pose": (
            pose_vector_json(guidance_start)
            if guidance_text and guidance_start is not None
            else None
        ),
        "round_index": round_index,
        "state": "initial" if round_index == 1 else "continuing",
        "previous_points_json": points_json(previous_trajectory) if has_history else None,
        "previous_start_pose_json": (
            pose_vector_json(previous_trajectory_start)
            if has_history and previous_trajectory_start is not None
            else None
        ),
        "measured": waypoint_values(
            world_pose_delta_in_reference_frame(start_pose, observation.pose)
        ),
        "pose": pose_values(observation.pose),
        "camera_json": compact_json(camera_values(observation)),
        "pending_json": points_json(pending_trajectory),
    }
