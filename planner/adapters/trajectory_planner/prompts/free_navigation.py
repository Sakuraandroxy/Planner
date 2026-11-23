from planner.adapters.prompts import render_prompt
from planner.adapters.prompts.values import free_space_profile_values
from planner.adapters.prompts.values import vehicle_values
from planner.algorithms.depth import (
    FreeSpaceProfile,
    FreeSpaceProfileSettings,
    measure_free_space,
)
from planner.domain import Observation
from planner.domain import RelativeTrajectory
from planner.domain import WorldPose
from planner.errors import ProtocolError

from .base import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    build_aerial_base_user_prompt,
    build_base_user_prompt,
)


def free_navigation_system_prompt(*, aerial: bool = False) -> str:
    """Return the UAV or ground-vehicle free-navigation system prompt."""
    base = AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT if aerial else BASE_TRAJECTORY_SYSTEM_PROMPT
    return render_prompt(
        "free_navigation", base_system=base.rstrip("\n"), **vehicle_values(aerial)
    )


FREE_NAVIGATION_SYSTEM_PROMPT = free_navigation_system_prompt()

# The reference Planner has no free-navigation prompt, so the drone keeps the
# same road-following procedure with flight wording on top of the UAV base.
AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT = free_navigation_system_prompt(aerial=True)


class FreeNavigationTrajectoryPrompt:
    """Target-free road-following prompt; the base prompt supplies the protocol."""

    def __init__(
        self,
        free_space: FreeSpaceProfileSettings | None = None,
        *,
        aerial: bool = False,
    ):
        self.free_space = free_space or FreeSpaceProfileSettings()
        self.aerial = aerial

    def system_prompt(self) -> str:
        return free_navigation_system_prompt(aerial=self.aerial)

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
        task_context: None = None,
    ) -> str:
        if task_context is not None:
            raise ProtocolError(
                "free navigation prompt does not support task context"
            )
        base = (build_aerial_base_user_prompt if self.aerial else build_base_user_prompt)(
            instruction,
            observation,
            max_points,
            current_guidance=current_guidance,
            round_index=round_index,
            initial_pose=initial_pose,
            previous_trajectory=previous_trajectory,
            pending_trajectory=pending_trajectory,
            guidance_start=guidance_start,
            previous_trajectory_start=previous_trajectory_start,
        )
        profile = measure_free_space(observation, self.free_space)
        return render_prompt(
            "free_navigation_user",
            base_user=base.rstrip("\n"),
            free_space_profile=format_free_space_profile(profile).rstrip("\n"),
        )


def format_free_space_profile(profile: FreeSpaceProfile) -> str:
    """Render the measured free-space profile block."""
    return render_prompt("free_space_profile", **free_space_profile_values(profile))
