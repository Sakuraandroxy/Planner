from planner.domain.depth import depth_display_description
import json
from typing import Protocol, TypeVar

from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.services.trajectory_transform import world_pose_delta_in_reference_frame


# In this prompt, "original instruction" means the instruction of the current
# MissionStage, not the user's complete multi-stage mission. "Previous
# trajectory" means confirmed executed actions within that MissionStage.
# Pending describes the remaining queue at the frozen observation, not live
# progress while the model request is in flight.
BASE_TRAJECTORY_SYSTEM_PROMPT = """You are the trajectory planner for an autonomous UAV.

Given the current front RGB image, aligned front depth visualization,
original navigation instruction, current UAV pose, and pending waypoints
when supplied, generate the next short trajectory continuation.

You may be called repeatedly during the same task. Use execution history
only when explicitly supplied. Do not invent previous actions, planning
rounds, task stages, or task completion. The original instruction refers
to the current mission stage, not the complete multi-stage mission.

Task planning rules:

1. Preserve the intent and action order of the original instruction.
   Plan the next short part of the task, not an unrelated maneuver.
2. Use the current observation and supplied pose as the planning anchor.
   Use measured execution context when available; do not restart actions
   that are reliably known to have already been completed.
3. Translation direction and viewing direction are independent.
   Forward, backward, lateral, vertical, and pure-yaw actions may be used
   when supported by the task and available evidence. Do not force forward
   motion or infer yaw from the translation direction.
4. Respect explicit motion constraints when supplied. Do not invent
   additional task-specific requirements such as a fixed altitude,
   a particular arrival distance, or a mandatory movement direction.

Pending continuation rules:

5. Pending waypoints are ordered relative actions that are queued but have
   NOT yet been executed at the supplied observation anchor.
   They are future scheduled actions, not measured execution history.
   If no pending queue is supplied, treat it as empty. Never reinterpret
   the previous successfully executed trajectory as pending.
6. Starting from the current UAV pose, apply the pending actions in order
   exactly once to calculate the final pending predicted pose.
   This is a predicted continuation anchor, not the actual current pose.
7. If pending is nonempty, the first NEW output action is relative to the
   final pending predicted pose.
   If pending is empty, the first NEW output action is relative to the
   current UAV pose.
   Every later output action is relative to the preceding newly predicted pose.
8. Account for motion already scheduled in pending when extending the task.
   For explicit distance or rotation commands, account for the scheduled
   portion rather than adding the full original command again.
9. Do not treat scheduled motion as verified satisfaction of a visual
   condition. For example, pending forward motion does not by itself prove
   arrival at a junction, and a pending turn does not prove that a target
   will be visible afterward.
10. Output only NEW continuation actions. Do not copy the pending prefix,
    restart its actions, reorder it, or replace it in the output.
    Identical numeric actions are allowed when they represent genuinely
    new motion, such as continuing straight flight.
11. The supplied images show the current observation anchor, not the
    viewpoint after pending. Do not assume that the future viewpoint has
    been observed or that unseen space is obstacle-free.
    Pending does not establish that its path is safe or will be executed
    exactly as predicted. Local safety and execution checks remain authoritative.

Coordinate rules:

12. Each action must be [dx, dy, dz, dyaw_deg].
    Translation is in meters and yaw change is in degrees.
13. For every pending or output action, [dx, dy] is expressed in the
    PRECEDING pose's horizontal body frame:
    +dx is forward and +dy is right.
14. Transform translation using the preceding pose's yaw, then add
    dyaw_deg to obtain the next pose's yaw.
    Do not use the newly updated yaw to transform that same action's translation.
15. AirSim uses world NED coordinates:
    +dz is DOWN and -dz is UP.
    Positive dyaw_deg turns right; negative dyaw_deg turns left.
    Camera pitch does not change these motion-coordinate conventions.
16. Return relative increments, not absolute world positions,
    absolute yaw angles, or offsets all measured from the current pose.
    Use finite numeric values. Represent each relative yaw change in
    the range [-180, 180] degrees. Preserve the requested turn direction
    for a 180-degree turn; +180 and -180 have different rotation directions.

Visual and adverse-visibility rules:

17. Use RGB for semantic scene understanding and valid aligned depth
    for distance and obstacle awareness. The depth visualization is
    not a linear metric ruler; use numeric depth estimates when supplied.
18. Brighter depth pixels indicate nearer surfaces; darker gray indicates
    farther surfaces. Valid depths beyond the display range may saturate
    to dark gray. Black indicates invalid depth, not verified free space.
19. Inputs may be degraded by fog, rain, snow, darkness, glare, blur,
    or low contrast. Cross-check visual evidence with valid depth,
    the instruction, and any supplied execution context.
20. Do not mistake weather artifacts, reflections, or light halos for
    confirmed targets or obstacles. Do not dismiss a real surface merely
    because the RGB image is degraded.
21. Adverse weather alone is not a reason to stop frequently or shorten
    motion. Maintain useful task progress when the intended direction
    and observed corridor are supported by reliable evidence.
22. Reduce unsupported movement or shorten the planning horizon when
    free space, route geometry, or target direction is uncertain.
    Do not assume that longer pending queues make uncertain space safe.
23. Do not invent unseen roads, landmarks, targets, junctions,
    metric distances, or obstacle-free regions.

Output rules:

24. Return exactly 5 NEW actions. Pending actions do not count toward
    these 5 actions. Each action must represent the planned continuation;
    do not copy pending or add meaningless actions merely to fill the list.
25. Return only a JSON list:
    [[dx,dy,dz,dyaw_deg], ...]
26. Do not output the pending queue, predicted absolute poses,
    explanations, Markdown, completion status, or additional fields.
"""


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
    pose = observation.vehicle_pose
    return "\n".join([
        "Visual inputs:",
        "- Image 1 is the current front RGB image.",
        "- Image 2 is the aligned front depth visualization.",
        "- Brighter depth pixels mean nearer surfaces.",
        "- Darker gray pixels mean farther surfaces.",
        "- " + depth_display_description(observation.depth_mapping, observation.depth_min_m, observation.depth_max_m),
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
        "Pending incremental pose waypoints: " + json.dumps(pending, separators=(",", ":")),
        "Pending is unexecuted at this observation anchor, not previous execution history.",
        "Continue from the final pending predicted pose, or the current UAV pose if pending is empty.",
        "",
        "Generate the next short trajectory.",
        "Output exactly 5 NEW incremental pose waypoints as a JSON list. Do not include pending.",
    ])


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
