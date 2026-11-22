from planner.domain.lane_reference import LaneReference
from planner.domain.observation import Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.errors import ProtocolError
from planner.services.free_space import (
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
        task_context: LaneReference | None = None,
    ) -> str:
        if task_context is not None and not isinstance(
            task_context, LaneReference
        ):
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
            + format_lane_reference(task_context)
            + "\n\n"
            + format_free_space_profile(profile)
        )


def format_lane_reference(reference: LaneReference | None) -> str:
    lines = [
        (
            "Lane reference (measured from the simulator HD map at the planning "
            "anchor for these new actions - the predicted pose after the pending "
            "queue, or the current pose when nothing is pending):"
        )
    ]
    if reference is None:
        lines.append(
            "- unavailable at the planning anchor (no aligned driving lane is "
            "projected, or the vehicle is inside a junction)."
        )
        lines.append(
            "- Fall back to the measured free-space profile and the visible "
            "road edges; keep to the current road and do not invent a lane."
        )
        return "\n".join(lines)
    lines.append(
        f"- Current lane: road {reference.road_id}, lane {reference.lane_id}, "
        f"width {reference.lane_width_m:.2f} m."
    )
    lines.append(
        f"- Lateral offset: {reference.lateral_offset_m:+.2f} m "
        "(positive is right of the lane centre). "
        f"Lane heading error: {reference.heading_error_deg:+.1f} deg "
        "(positive means the lane points right of the car)."
    )
    lines.append(
        f"- Signed distance to the left lane boundary: "
        f"{reference.distance_to_left_boundary_m:.2f} m; to the right lane "
        f"boundary: {reference.distance_to_right_boundary_m:.2f} m "
        "(negative means the vehicle is outside that boundary)."
    )
    if reference.lookahead:
        lines.append("- Lane-centre lookahead in the current body frame:")
        for point in reference.lookahead:
            lines.append(
                f"  {point.distance_m:.1f} m ahead: "
                f"forward {point.forward_m:.1f} m, "
                f"right {point.right_m:+.1f} m, "
                f"bearing {_signed_deg(point.bearing_deg)} deg, "
                f"cumulative lane yaw {_signed_deg(point.yaw_delta_deg)} deg"
            )
        near = reference.lookahead[0]
        reach = min(
            reference.lookahead,
            key=lambda point: abs(point.distance_m - 25.0),
        )
        nearest_boundary_m = min(
            reference.distance_to_left_boundary_m,
            reference.distance_to_right_boundary_m,
        )
        lines.append(
            f"- Steering aim at the anchor: the {near.distance_m:.1f} m "
            f"lane-centre point is at bearing {_signed_deg(near.bearing_deg)} "
            f"deg (its tangent is {_signed_deg(near.yaw_delta_deg)} deg). "
            "Aim only the first one or two actions at this bearing; it "
            "corrects the lateral offset. Matching only the tangent keeps the "
            "car parallel but off-centre."
        )
        lines.append(
            f"- Final-heading check: the sum of the five dyaw values must end "
            f"within about 5 deg of the cumulative lane yaw at the reach, "
            f"{_signed_deg(reach.yaw_delta_deg)} deg "
            f"(measured at {reach.distance_m:.1f} m), not at the near-point "
            "bearing. The near bearing is only the early aim and the later "
            "actions must unwind to this reach tangent. A total that matches "
            "the near bearing instead points the car across the lane and makes "
            "the next plan reverse it (left-right weave)."
        )
        if (
            nearest_boundary_m < 1.0
            or abs(reference.heading_error_deg) > 20.0
        ):
            lines.append(
                f"- Tight space at the anchor: nearest lane boundary "
                f"{nearest_boundary_m:.2f} m, heading error "
                f"{_signed_deg(reference.heading_error_deg)} deg, lateral "
                f"offset {reference.lateral_offset_m:+.2f} m. Because each "
                "step first moves along the current heading, it drifts "
                "sideways by about dx * sin(heading error); keep that drift "
                "smaller than the nearer boundary distance and start with "
                "one or two short pivot-like steps (about 1 m) before "
                "lengthening the later steps."
            )
        else:
            lines.append(
                f"- Open lane at the anchor: nearest lane boundary "
                f"{nearest_boundary_m:.1f} m, heading error "
                f"{_signed_deg(reference.heading_error_deg)} deg, lateral "
                f"offset {reference.lateral_offset_m:+.2f} m. Use normal "
                "4-5 m forward steps for all five actions (reach about "
                "20-25 m). Do not copy the short 1-2 m first step from the "
                "tight-space or large-heading-error examples; here it would "
                "drain the waypoint queue."
            )
    else:
        lines.append(
            "- No lane-centre lookahead is available (the lane ends or "
            "branches within the report range)."
        )
    if reference.junction_ahead:
        lines.append(
            "- The current lane ends or reaches a junction within the "
            "lookahead; follow the road's natural continuation there and do "
            "not use the widest gap."
        )
    lines.append(
        "- Follow this measured lane centreline as the primary lateral guide: "
        "aim at the near lane-centre bearing before settling onto the reach "
        "tangent, and keep every predicted pose inside the lane boundaries."
    )
    lines.append(
        "- The anchor is where your five new actions begin, so every value "
        "above already includes the effect of the pending actions. Do not "
        "subtract or re-apply pending yaw or translation."
    )
    lines.append(
        "- Sign check: when the car is off-centre or the near-point bearing is "
        "clearly nonzero, the first one or two actions must turn toward the "
        "near-point bearing (positive right, negative left); the later actions "
        "must unwind so the total is within about 5 deg of the reach tangent "
        "above. Never mirror the sign and never use the near-point bearing as "
        "the total turn."
    )
    return "\n".join(lines)


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