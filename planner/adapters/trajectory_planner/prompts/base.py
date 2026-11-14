import json
from typing import Protocol

from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.services.trajectory_transform import world_pose_delta_in_reference_frame


# In this prompt, "original instruction" means the instruction of the current
# MissionStage, not the user's complete multi-stage mission. "Previous
# trajectory" means the previous commanded trajectory within that MissionStage.
BASE_TRAJECTORY_SYSTEM_PROMPT = """You are the trajectory planner for an autonomous UAV.

Given the current front RGB image, aligned front depth image, the original
navigation instruction, and the recent execution history, generate the next
short trajectory.

You may be called multiple times during the same navigation task. The current
request may represent the beginning or a continuation of the task.

Use the original instruction, current observation, current pose, cumulative
motion, and previous trajectory to infer what has already been done and what
should be done next.

Planning rules:

1. Preserve the intent and action order of the original instruction.
2. Plan only the next short part of the task.
3. Do not restart actions that have already been completed.
4. On the first planning round, begin from the current observation.
5. On later rounds, continue from the actual current observation and pose.
6. Use the RGB image for semantic scene understanding.
7. Use the aligned depth image for obstacle and distance awareness.
8. Translation direction and viewing direction may be different.
9. Generate between 1 and 5 incremental trajectory points.
10. Each point must be [dx, dy, dz, dyaw_deg].
11. The first point is relative to the current UAV pose.
12. Every following point is relative to the preceding predicted pose.
13. +dx means forward relative to the reference heading.
14. +dy means right relative to the reference heading.
15. AirSim uses NED coordinates: +dz is downward and -dz is upward.
16. +dyaw_deg turns right; -dyaw_deg turns left.
17. Output only the JSON waypoint list.
18. Do not output explanations, status text, Markdown, or additional fields.

Adverse-visibility rules:

19. Visual inputs may be degraded by fog, rain, snow, darkness, glare, blur,
    or low contrast.
20. Treat weather artifacts as visual noise; do not mistake rain streaks,
    snowflakes, fog, reflections, or light halos for targets or obstacles.
21. Cross-check RGB semantic evidence with valid aligned depth, the original
    instruction, the current pose, and recent execution history.
22. Adverse weather alone is not a reason to shorten the trajectory or stop
    frequently. When valid depth indicates a clear traversable corridor,
    maintain useful forward progress and a smooth planning horizon. Reduce only
    lateral, vertical, or yaw changes that are not supported by reliable visual
    or depth evidence.
23. Shorten the planned motion only when the free space, route geometry, or
    target direction cannot be established reliably. Do not reduce waypoint
    count or displacement solely because the scene contains fog, rain, snow,
    darkness, glare, or blur.
24. Do not invent an unseen road, landmark, target, junction, or free space.
    Continue only from evidence available in the current observation and
    execution context.

Required output:

[[dx,dy,dz,dyaw_deg], ...]"""


def build_base_user_prompt(
    instruction: str,
    observation: Observation,
    max_points: int,
    *,
    current_guidance: str | None = None,
    round_index: int = 1,
    initial_pose: WorldPose | None = None,
    previous_trajectory: RelativeTrajectory | None = None,
) -> str:
    """Build the baseline dynamic user prompt shared by trajectory tasks."""
    start_pose = initial_pose or observation.vehicle_pose
    measured = world_pose_delta_in_reference_frame(start_pose, observation.vehicle_pose)
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
    elif round_index == 1:
        guidance = "No additional guidance. Begin the original navigation instruction from the current observation."
    else:
        guidance = (
            "No additional guidance. Continue the original navigation instruction from the current "
            "observation without restarting completed actions."
        )
    history = (
        "No previous trajectory has been executed."
        if previous_trajectory is None
        else "Previous successfully executed trajectory from the preceding planning round "
        "(do not execute it again): " + json.dumps(
            [[point.dx, point.dy, point.dz, point.dyaw_deg] for point in previous_trajectory.points],
            separators=(",", ":"),
        )
    )
    pose = observation.vehicle_pose
    return "\n".join([
        "Visual inputs:",
        "- Image 1 is the current front RGB image.",
        "- Image 2 is the aligned front depth visualization.",
        "- Brighter depth pixels mean nearer surfaces.",
        "- Darker gray pixels mean farther surfaces.",
        "- Black pixels mean invalid depth.",
        "",
        "Original navigation instruction for the current mission stage:",
        instruction.strip(),
        "",
        "Current guidance:",
        guidance,
        "",
        "Execution context:",
        f"- Planning round: {round_index}",
        f"- Task state: {state}",
        f"- {history}",
        "- Measured net pose change from the mission-stage start, expressed in its horizontal body frame:",
        f"  - Forward: {measured.dx:.3f} m",
        f"  - Right: {measured.dy:.3f} m",
        f"  - NED down: {measured.dz:.3f} m",
        f"  - Yaw change: {measured.dyaw_deg:.3f} degrees",
        "",
        "Current UAV pose:",
        f"- World position: [{pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}]",
        f"- Yaw: {pose.yaw_deg:.3f} degrees",
        "",
        "Camera geometry (JSON): " + json.dumps(camera, separators=(",", ":")),
        "",
        "Generate the next short trajectory.",
        f"Output only 1 to {max_points} relative trajectory points.",
    ])


class TrajectoryPromptBuilder(Protocol):
    def system_prompt(self) -> str: ...

    def build_user_prompt(
        self,
        instruction: str,
        observation: Observation,
        max_points: int,
        *,
        current_guidance: str | None = None,
        round_index: int = 1,
        initial_pose: WorldPose | None = None,
        previous_trajectory: RelativeTrajectory | None = None,
    ) -> str: ...


class ProgressPromptBuilder(Protocol):
    def build(
        self,
        instruction: str,
        initial: Observation,
        current: Observation,
        round_index: int,
    ) -> str: ...
