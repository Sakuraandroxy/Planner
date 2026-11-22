import json
from typing import Protocol, TypeVar

from planner.domain import depth_display_description
from planner.domain import Observation
from planner.domain import WorldPose
from planner.domain import RelativeTrajectory
from planner.algorithms.trajectory_transform import world_pose_delta_in_reference_frame

from .render import load, render

# In this prompt, "original instruction" means the instruction of the current
# MissionStage, not the user's complete multi-stage mission. "Previous
# trajectory" means confirmed executed actions within that MissionStage.
# Pending describes the remaining queue at the frozen observation, not live
# progress while the model request is in flight.
BASE_TRAJECTORY_SYSTEM_PROMPT = load("base_system")


# Trajectory prompt for the drone, ported verbatim from the reference UAV
# Planner (/roxy/zhusai/Planner). The rover keeps the ground-vehicle prompt
# above; the two differ only in motion semantics, not in the output protocol.
AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT = load("aerial_base_system")


def base_system_prompt(*, aerial: bool = False) -> str:
    """Return the UAV or ground-vehicle baseline system prompt.

    The drone arm uses the reference Planner's UAV wording so vertical motion
    stays available; every ground-vehicle caller keeps the carla-vln wording.
    """
    if aerial:
        return AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT
    return BASE_TRAJECTORY_SYSTEM_PROMPT


def build_base_user_prompt(
    instruction: str,
    observation: Observation,
    max_points: int,
    *,
    current_guidance: str | None = None,
    guidance_start: WorldPose | None = None,
    round_index: int = 1,
    initial_pose: WorldPose | None = None,
    previous_trajectory: RelativeTrajectory | None = None,
    previous_trajectory_start: WorldPose | None = None,
    pending_trajectory: RelativeTrajectory | None = None,
) -> str:
    """Build the baseline dynamic user prompt shared by trajectory tasks."""
    if max_points != 5:
        raise ValueError("trajectory prompt requires exactly 5 new points")
    if pending_trajectory is not None and not isinstance(pending_trajectory, RelativeTrajectory):
        raise TypeError("pending_trajectory must be a RelativeTrajectory or None")
    pending = [] if pending_trajectory is None else [
        [point.dx, point.dy, point.dz, point.dyaw_deg] for point in pending_trajectory.points
    ]
    if len(pending) > 5:
        raise ValueError("pending queue must contain 0 to 5 points")
    start_pose = initial_pose or observation.pose
    measured = world_pose_delta_in_reference_frame(start_pose, observation.pose)
    camera = {
        "camera_id": observation.camera_id,
        "horizontal_fov_deg": round(observation.intrinsics.horizontal_fov_deg, 3),
        "camera_position_world": [round(value, 4) for value in observation.camera_position_world],
        "rotation_camera_to_world": [
            [round(value, 6) for value in row]
            for row in observation.rotation_camera_to_world
        ],
    }
    state = "initial" if round_index == 1 else "continuing"
    if current_guidance and current_guidance.strip():
        guidance = current_guidance.strip()
    else:
        guidance = "Not supplied."
    if current_guidance and guidance_start is not None:
        guidance += "\nGuidance reference vehicle pose [world x,y,z,yaw_deg]: " + json.dumps(
            [guidance_start.x, guidance_start.y, guidance_start.z, guidance_start.yaw_deg],
            separators=(",", ":"))
        guidance += (
            "\nThis remaining-action guidance was issued at that reference pose, not renewed at the "
            "current observation. Account for motion executed since that pose and for pending "
            "before generating NEW continuation actions."
        )
    history = (
        ("No previous trajectory has been executed." if round_index == 1 else
         "Execution history is not supplied; do not infer that no motion has occurred.")
        if previous_trajectory is None
        else "Recent confirmed executed actions, not a complete previous planning batch "
        "(do not execute it again): " + json.dumps(
            [[point.dx, point.dy, point.dz, point.dyaw_deg] for point in previous_trajectory.points],
            separators=(",", ":"),
        )
    )
    if previous_trajectory is not None and previous_trajectory_start is not None:
        history += "\n  Execution-history reference start pose: " + json.dumps(
            [previous_trajectory_start.x, previous_trajectory_start.y,
             previous_trajectory_start.z, previous_trajectory_start.yaw_deg], separators=(",", ":"))
    pose = observation.pose
    return render(
        load("base_user_prompt"),
        depth_protocol=depth_display_description(
            observation.depth_mapping, observation.depth_min_m, observation.depth_max_m),
        instruction=instruction.strip(),
        guidance=guidance,
        round_index=round_index,
        state=state,
        history=history,
        measured_dx=f"{measured.dx:.3f}",
        measured_dy=f"{measured.dy:.3f}",
        measured_dz=f"{measured.dz:.3f}",
        measured_dyaw=f"{measured.dyaw_deg:.3f}",
        pose_x=f"{pose.x:.3f}",
        pose_y=f"{pose.y:.3f}",
        pose_z=f"{pose.z:.3f}",
        pose_yaw=f"{pose.yaw_deg:.3f}",
        camera_json=json.dumps(camera, separators=(",", ":")),
        pending_json=json.dumps(pending, separators=(",", ":")),
    ).removesuffix("\n")


def build_aerial_base_user_prompt(
    instruction: str,
    observation: Observation,
    max_points: int,
    *,
    current_guidance: str | None = None,
    guidance_start: WorldPose | None = None,
    round_index: int = 1,
    initial_pose: WorldPose | None = None,
    previous_trajectory: RelativeTrajectory | None = None,
    previous_trajectory_start: WorldPose | None = None,
    pending_trajectory: RelativeTrajectory | None = None,
) -> str:
    """Build the UAV dynamic user prompt, ported verbatim from the reference Planner.

    The reference stores the robot pose as ``observation.vehicle_pose``; this
    project stores the same data as ``observation.pose``. Everything else is
    the reference wording, so the drone arm matches /roxy/zhusai/Planner.
    """
    if max_points != 5:
        raise ValueError("trajectory prompt requires exactly 5 new points")
    if pending_trajectory is not None and not isinstance(pending_trajectory, RelativeTrajectory):
        raise TypeError("pending_trajectory must be a RelativeTrajectory or None")
    pending = [] if pending_trajectory is None else [
        [point.dx, point.dy, point.dz, point.dyaw_deg] for point in pending_trajectory.points
    ]
    if len(pending) > 5:
        raise ValueError("pending queue must contain 0 to 5 points")
    start_pose = initial_pose or observation.pose
    measured = world_pose_delta_in_reference_frame(start_pose, observation.pose)
    camera = {
        "camera_id": observation.camera_id,
        "horizontal_fov_deg": round(observation.intrinsics.horizontal_fov_deg, 3),
        "camera_position_world": [round(value, 4) for value in observation.camera_position_world],
        "rotation_camera_to_world": [
            [round(value, 6) for value in row]
            for row in observation.rotation_camera_to_world
        ],
    }
    state = "initial" if round_index == 1 else "continuing"
    if current_guidance and current_guidance.strip():
        guidance = current_guidance.strip()
    else:
        guidance = "Not supplied."
    if current_guidance and guidance_start is not None:
        guidance += "\nGuidance reference UAV pose [world x,y,z,yaw_deg]: " + json.dumps(
            [guidance_start.x, guidance_start.y, guidance_start.z, guidance_start.yaw_deg],
            separators=(",", ":"))
        guidance += (
            "\nThis remaining-action guidance was issued at that reference pose, not renewed at the "
            "current observation. Account for motion executed since that pose and for pending "
            "before generating NEW continuation actions."
        )
    history = (
        ("No previous trajectory has been executed." if round_index == 1 else
         "Execution history is not supplied; do not infer that no motion has occurred.")
        if previous_trajectory is None
        else "Recent confirmed executed actions, not a complete previous planning batch "
        "(do not execute it again): " + json.dumps(
            [[point.dx, point.dy, point.dz, point.dyaw_deg] for point in previous_trajectory.points],
            separators=(",", ":"),
        )
    )
    if previous_trajectory is not None and previous_trajectory_start is not None:
        history += "\n  Execution-history reference start pose: " + json.dumps(
            [previous_trajectory_start.x, previous_trajectory_start.y,
             previous_trajectory_start.z, previous_trajectory_start.yaw_deg], separators=(",", ":"))
    pose = observation.pose
    return render(
        load("aerial_base_user_prompt"),
        depth_protocol=depth_display_description(
            observation.depth_mapping, observation.depth_min_m, observation.depth_max_m),
        instruction=instruction.strip(),
        guidance=guidance,
        round_index=round_index,
        state=state,
        history=history,
        measured_dx=f"{measured.dx:.3f}",
        measured_dy=f"{measured.dy:.3f}",
        measured_dz=f"{measured.dz:.3f}",
        measured_dyaw=f"{measured.dyaw_deg:.3f}",
        pose_x=f"{pose.x:.3f}",
        pose_y=f"{pose.y:.3f}",
        pose_z=f"{pose.z:.3f}",
        pose_yaw=f"{pose.yaw_deg:.3f}",
        camera_json=json.dumps(camera, separators=(",", ":")),
        pending_json=json.dumps(pending, separators=(",", ":")),
    ).removesuffix("\n")


ContextT = TypeVar("ContextT", contravariant=True)


class TrajectoryPromptBuilder(Protocol[ContextT]):
    def system_prompt(self) -> str: ...

    def build_user_prompt(
        self,
        instruction: str,
        observation: Observation,
        max_points: int,
        *,
        current_guidance: str | None = None,
        guidance_start: WorldPose | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
        previous_trajectory_start: WorldPose | None = None,
        pending_trajectory: RelativeTrajectory | None = None,
        task_context: ContextT | None = None,
    ) -> str: ...


class BaseTrajectoryPrompt:
    """Concrete default builder; task-specific builders remain explicitly injected."""
    def system_prompt(self) -> str:
        return BASE_TRAJECTORY_SYSTEM_PROMPT

    def build_user_prompt(
        self, instruction: str, observation: Observation, max_points: int, *,
        current_guidance: str | None = None, guidance_start: WorldPose | None = None, round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
        previous_trajectory_start: WorldPose | None = None,
        pending_trajectory: RelativeTrajectory | None = None,
        task_context: None = None,
    ) -> str:
        if task_context is not None:
            from planner.errors import ProtocolError
            raise ProtocolError("base trajectory prompt does not support task context")
        return build_base_user_prompt(
            instruction, observation, max_points, current_guidance=current_guidance,
            round_index=round_index, initial_pose=initial_pose, previous_trajectory=previous_trajectory,
            pending_trajectory=pending_trajectory,
            guidance_start=guidance_start,
            previous_trajectory_start=previous_trajectory_start,
        )