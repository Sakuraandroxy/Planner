"""Trajectory prompt builders for the ground vehicle and the UAV.

System text comes from ``trajectory_ground``/``trajectory_aerial`` and user text
from ``trajectory_refine``; the values those templates read come from
``planner.adapters.prompts.values``.

In this prompt, "original instruction" means the instruction of the current
MissionStage, not the user's complete multi-stage mission. "Previous trajectory"
means confirmed executed actions within that MissionStage. Pending describes the
remaining queue at the frozen observation, not live progress while the model
request is in flight.
"""

from typing import Protocol, TypeVar

from planner.adapters.prompts import render_prompt
from planner.adapters.prompts.values import trajectory_refine_values
from planner.domain import Observation
from planner.domain import RelativeTrajectory
from planner.domain import WorldPose


BASE_TRAJECTORY_SYSTEM_PROMPT = render_prompt("trajectory_ground")

# The drone arm keeps the reference UAV wording, so vertical motion stays
# available; every ground-vehicle caller keeps the carla-vln wording.
AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT = render_prompt("trajectory_aerial")


def base_system_prompt(*, aerial: bool = False) -> str:
    """Return the UAV or ground-vehicle baseline system prompt."""
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
    return _user_prompt(
        instruction,
        observation,
        max_points,
        aerial=False,
        current_guidance=current_guidance,
        guidance_start=guidance_start,
        round_index=round_index,
        initial_pose=initial_pose,
        previous_trajectory=previous_trajectory,
        previous_trajectory_start=previous_trajectory_start,
        pending_trajectory=pending_trajectory,
    )


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
    """Build the UAV dynamic user prompt, in the reference Planner's wording."""
    return _user_prompt(
        instruction,
        observation,
        max_points,
        aerial=True,
        current_guidance=current_guidance,
        guidance_start=guidance_start,
        round_index=round_index,
        initial_pose=initial_pose,
        previous_trajectory=previous_trajectory,
        previous_trajectory_start=previous_trajectory_start,
        pending_trajectory=pending_trajectory,
    )


def _user_prompt(
    instruction: str,
    observation: Observation,
    max_points: int,
    *,
    aerial: bool,
    current_guidance: str | None,
    guidance_start: WorldPose | None,
    round_index: int,
    initial_pose: WorldPose | None,
    previous_trajectory: RelativeTrajectory | None,
    previous_trajectory_start: WorldPose | None,
    pending_trajectory: RelativeTrajectory | None,
) -> str:
    """Validate one planning round, then render ``trajectory_refine``."""
    if max_points != 5:
        raise ValueError("trajectory prompt requires exactly 5 new points")
    if pending_trajectory is not None and not isinstance(pending_trajectory, RelativeTrajectory):
        raise TypeError("pending_trajectory must be a RelativeTrajectory or None")
    if pending_trajectory is not None and len(pending_trajectory.points) > 5:
        raise ValueError("pending queue must contain 0 to 5 points")
    return render_prompt(
        "trajectory_refine",
        **trajectory_refine_values(
            instruction,
            observation,
            aerial=aerial,
            current_guidance=current_guidance,
            guidance_start=guidance_start,
            round_index=round_index,
            initial_pose=initial_pose,
            previous_trajectory=previous_trajectory,
            previous_trajectory_start=previous_trajectory_start,
            pending_trajectory=pending_trajectory,
        ),
    )


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
