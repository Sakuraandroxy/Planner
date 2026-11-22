from planner.domain import Observation
from planner.domain import WorldPose
from planner.domain import RelativeTrajectory
from planner.errors import ProtocolError
from planner.algorithms import (
    FreeSpaceProfile,
    FreeSpaceProfileSettings,
    measure_free_space,
)

from .base import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    build_aerial_base_user_prompt,
    build_base_user_prompt,
)
from .render import load

FREE_NAVIGATION_SYSTEM_PROMPT = (
    BASE_TRAJECTORY_SYSTEM_PROMPT + "\n\n" + load("free_navigation_rules")
)

# The reference Planner has no free-navigation prompt, so the drone keeps the
# same road-following procedure with flight wording on top of the UAV base.
AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT = (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT + "\n\n" + load("aerial_free_navigation_rules")
)


def free_navigation_system_prompt(*, aerial: bool = False) -> str:
    """Return the UAV or ground-vehicle free-navigation system prompt."""
    if aerial:
        return AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT
    return FREE_NAVIGATION_SYSTEM_PROMPT


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
        return (
            base
            + "\n\n"
            + format_free_space_profile(profile)
        )


def format_free_space_profile(profile: FreeSpaceProfile) -> str:
    lines = [
        "Metric free-space profile (measured from the aligned metric depth at this observation):",
        "- Units are metres in the current body horizontal frame; bearing is measured from the current forward axis and is positive to the right.",
        f"- Each sector lists the nearest surface at least {profile.min_height_above_ground_m:.2f} m above the local ground, within {profile.max_range_m:.1f} m.",
    ]
    for sector in profile.sectors:
        label = _bearing_label(sector.bearing_deg)
        if not sector.ground_available:
            lines.append(f"  {label} deg: unknown (no ground reference in view)")
        elif sector.distance_m is None:
            lines.append(
                f"  {label} deg: clear beyond {profile.max_range_m:.1f} m"
            )
        else:
            lines.append(
                f"  {label} deg: {sector.distance_m:.1f} m "
                f"(forward {sector.forward_m:.1f} m, "
                f"right {sector.right_m:+.1f} m)"
            )
    ahead = min(
        profile.sectors, key=lambda sector: abs(sector.bearing_deg), default=None
    )
    if ahead is not None:
        label = _bearing_label(ahead.bearing_deg)
        if not ahead.ground_available:
            lines.append(
                f"- Nearest obstacle directly ahead ({label} deg): unknown."
            )
        elif ahead.distance_m is None:
            lines.append(
                f"- Nearest obstacle directly ahead ({label} deg): clear "
                f"beyond {profile.max_range_m:.1f} m."
            )
        else:
            lines.append(
                f"- Nearest obstacle directly ahead ({label} deg): "
                f"{ahead.distance_m:.1f} m."
            )
    window = profile.clear_window_deg
    if window is not None:
        lines.append(
            "- Longest clear bearing window: "
            f"{_bearing_label(window[0])} to {_bearing_label(window[1])} deg."
        )
    lines.append(
        "Use these measured distances and bearings instead of estimating scale "
        "from the RGB image. A clear sector has no measured above-ground surface "
        "inside the report range; it does not prove unseen space is obstacle-free."
    )
    return "\n".join(lines)


def _bearing_label(value: float) -> str:
    return _signed_deg(value)


def _signed_deg(value: float) -> str:
    return "0.0" if abs(value) < 0.05 else f"{value:+.1f}"