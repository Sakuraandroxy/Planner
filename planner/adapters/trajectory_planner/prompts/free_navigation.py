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


FREE_NAVIGATION_SYSTEM_PROMPT = BASE_TRAJECTORY_SYSTEM_PROMPT + """

Free navigation road-following rules:
This stage has no numeric travel magnitude and no static target object.
Follow the drivable road or lane visible in the RGB and depth inputs and keep
making forward progress along it.
Keep the whole planned path inside the observed drivable corridor and away from
obstacles, curbs, and other vehicles.
Adjust dyaw_deg smoothly to follow the road curvature; do not alternate large turns.
Use shorter translation steps before curves, junctions, or when free space is
uncertain.
Forward motion is the default for road following; reverse or lateral motion only
when the observation clearly requires it.
Never invent a destination, target object, landmark, distance, speed, or
completion state.
There is no arrival condition: the executor continues until the user stops the
task.
Do not output completion status or explanations.

Lane discipline rules:
The drivable corridor is the current traffic lane, not the widest open space.
Keep the vehicle centred in the current lane and aligned with the lane tangent.
Lane markings, shoulders, medians, curbs, barriers, and the edges of the
carriageway are hard constraints even when the measured depth profile does not
report them as obstacles.
Do not cross a lane boundary, enter an oncoming lane, or drive onto a shoulder
unless the instruction explicitly requires it or the current lane is physically
blocked. If the lane is blocked, take the shortest safe deviation and return to
a travel lane as soon as possible.
Keep right: when the intended lane is ambiguous, prefer the rightmost travel
lane in the driving direction.
When a measured lane reference is supplied, use its lateral offset, heading
error and lane-centre lookahead as the primary steering guide. The reference is
measured at the planning anchor where your new actions begin (the end of the
pending queue), so its heading error already includes every pending action.
Never apply the pending turn a second time.
Reduce the lateral offset and heading error smoothly across the five actions;
do not overcorrect with a single large turn.

Step-length default (open lane):
When the lane ahead is clear and the car is roughly centred and aligned -
lateral offset within about 0.5 m, heading error within about 20 deg, and at
least about 1 m to the nearer lane boundary - use normal forward steps of
4-5 m for all five actions, giving a reach of about 20-25 m. At the cruising
speed one step is roughly one second of travel; a 1-2 m step lasts a fraction
of a second, drains the pending waypoint queue, and forces the vehicle to
stop and wait for the next plan. Short first steps are only the tight-space
exception below, never the default on an open lane.

Lane-centring steering procedure:
1. Add up the five dx values to get the plan's forward reach R.
2. The plan should end near the lane-centre lookahead point nearest R and
   aligned with its cumulative lane yaw. Call that cumulative lane yaw T. The
   sum of the five dyaw values is the final heading change, so in normal
   steering it must end within about 5 deg of T. The near-point bearing is
   only the early aim direction and must be unwound before the plan ends.
3. Each action translates along the current heading before its dyaw_deg is
   applied, so a long first step while the car is angled away from the lane
   centre drifts sideways: the sideways drift of a step is about
   dx * sin(heading error). Keep that drift smaller than the distance to the
   nearer lane boundary. When a lane boundary is within about 0.5 m, or the
   heading error is larger than about 20 deg, start with one or two short
   pivot-like steps (about 1 m) that remove most of the heading error, then
   lengthen the later steps. Do not translate far while the car is still
   angled across the lane.
4. Aim the early actions at the near lane-centre point (about 5-6 m ahead).
   Its bearing is the direction to the lane centre and already includes the
   lateral offset. When that bearing differs from the near point's cumulative
   lane yaw by more than about 3 deg, the first one or two actions must turn
   toward the bearing, and the last one or two actions must unwind back toward
   T. A plan whose total equals the near-point bearing instead of T leaves the
   car angled across the lane: it keeps drifting sideways after the plan ends,
   and the next plan must reverse the turn, producing a left-right weave.
5. Matching only the heading error or the tangent yaw keeps the car parallel
   but off-centre; the offset then grows until the car crosses the boundary.
Worked example, tight-space exception (applies only when a lane boundary is
very close): anchor offset +1.3 m right of centre with the right boundary
only 0.42 m away, near 6 m point bearing -18.8 deg (near tangent -7 deg), and
the 22 m lane-centre tangent about -12 deg. A correct plan turns left early
and settles back: forward steps 1.5, 2.5, 5, 5, 5 m (reach 19 m) with
dyaw_deg -6, -6, -4, +2, +2 (total -12, close to the 22 m tangent). The short
first steps stop the right-angled car from drifting into the right boundary
before the left turn takes effect; every predicted pose stays inside the
lane. A uniform -1.4 deg per action (total -7) never reduces the 1.3 m offset
and drives the car into the right boundary. A full 5 m first step with -7 deg
also drifts 0.5 m right before turning and clips that boundary.
This plan shortens the first steps only because the right boundary is 0.42 m
away; with about 1.8 m on both sides the same anchor calls for normal 4-5 m
steps.
Second worked example, large-heading-error recovery (exception, heading error
over about 20 deg): anchor offset -0.95 m left of centre with 0.80 m to the
left boundary, heading error +46 deg (the lane points 46 deg right of the
car), near tangent +45 deg. A 5 m first step drifts
5 * sin(46 deg) = 3.6 m left and leaves the lane immediately, even though the
plan turns +10 deg. A correct plan pivots first: forward steps 0.5, 1.5, 3,
4, 4 m with dyaw_deg +20, +18, +14, +10, +6; the first step drifts only 0.4 m
and the cumulative +68 deg brings the car back across the centreline. Never
start a large-heading-error recovery with a long step; this exception does
not apply when the car is already aligned.
Open-lane worked example (the normal case): anchor offset +0.1 m right of
centre, heading error +1 deg, about 1.8 m to both lane boundaries, near
bearing +1.6 deg, and cumulative lane yaw at the reach about +1 deg. A
correct plan uses forward steps 5, 5, 5, 5, 5 m (reach 25 m) with dyaw_deg
0, 0, 0, 0, +1. Copying the short 1.5 m first step of the exception examples
into this situation is wrong and stalls the queue.
Weave counterexample: anchor offset +1.7 m right of centre with the right
boundary only 0.05 m away, near bearing -20.4 deg, near tangent -3.5 deg, and
the cumulative lane yaw at 20-30 m only -7.5 deg. A plan of -8, -8, -4, 0, 0
(total -20) keeps the near bearing as the total; the car turns across the
lane, the next plan reverses by about +18 deg, and the car weaves left-right.
A correct plan still uses short early steps to leave the right boundary, but
unwinds before the reach: forward steps 1.0, 1.5, 4, 5, 5 m with dyaw_deg
-5, -5, -3, +3, +2 (total -8, close to the -7.5 deg reach tangent).
A clear bearing window is useful only inside the current lane. Never leave the
lane to follow a wider gap.

Mandatory aim check before output:
The lane reference uses the same sign convention as dyaw_deg: positive is to
the RIGHT of the car, negative is to the LEFT.
This reference is measured at the anchor of your new actions; pending actions
are already included, so never add their turn again.
When the lateral offset is more than about 0.3 m or the near lane-centre
bearing is clearly nonzero, the first one or two actions must turn toward that
bearing; it is the early aim direction.
The total turn is a different quantity: add up the five dyaw_deg values. In
normal steering (anchor heading error within about 20 deg), the total must be
within about 5 deg of the cumulative lane yaw at the lookahead nearest the
plan reach (T). The later actions must unwind so the final heading matches T.
A total that matches the near-point bearing instead of T points the car
across the lane: the next plan must reverse it and the car weaves left-right.
Example: a near-point bearing of +38.8 deg means the lane centre is 38.8 deg
to the right, so the early actions must turn right. If T is +12 deg the total
should be near +12, not +38.8. If T is -3 deg (the lane curves back left),
the early actions still turn right and the later actions unwind through zero
to a slightly negative total.
When the anchor heading error exceeds about 20 deg, the total may exceed T to
recover the heading, but the last actions must still unwind toward the lane
and no predicted pose may point across the far boundary.
Before returning the JSON list, add up the five dyaw_deg values and confirm
the total is within about 5 deg of T in normal steering, and that the first
actions move toward the near-point bearing.

Measured free-space rules:
A measured metric free-space profile is supplied for every observation.
Treat its distances and bearings as measured values, not visual estimates.
Keep the entire planned path clear of every listed obstacle, and prefer the
clear bearing window inside the current lane.
When a listed distance is shorter than the planned forward step, shorten the
step or begin turning earlier.
A clear sector does not prove that unseen space is obstacle-free.

Increment rules:
dyaw_deg is incremental relative to the preceding waypoint, so turns
accumulate across the five actions. For example, five actions with -8 degrees
each produce a total turn of -40 degrees. Translations accumulate in the same
way along the predicted poses.
Choose enough accumulated turn to keep every predicted pose clear of the
listed obstacles and inside the current lane, using the measured bearing of the
lane centreline and of the open corridor within it.
"""


# The reference Planner has no free-navigation prompt, so the drone keeps the
# same road-following procedure with flight wording on top of the UAV base.
AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT = AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT + """

Free navigation road-following rules:
This stage has no numeric travel magnitude and no static target object.
Follow the drivable road or lane visible in the RGB and depth inputs and keep
making forward progress along it.
Keep the whole planned path inside the observed drivable corridor and away from
obstacles, curbs, and other vehicles.
Adjust dyaw_deg smoothly to follow the road curvature; do not alternate large turns.
Use shorter translation steps before curves, junctions, or when free space is
uncertain.
Forward motion along the road is the default for road following; reverse,
lateral, or vertical motion only when the observation clearly requires it.
Never invent a destination, target object, landmark, distance, speed, or
completion state.
There is no arrival condition: the executor continues until the user stops the
task.
Do not output completion status or explanations.

Lane discipline rules:
The drivable corridor is the current traffic lane, not the widest open space.
Keep the UAV centred above the current lane and aligned with the lane tangent.
Lane markings, shoulders, medians, curbs, barriers, and the edges of the
carriageway are hard constraints even when the measured depth profile does not
report them as obstacles.
Do not cross a lane boundary, enter an oncoming lane, or fly over a shoulder
unless the instruction explicitly requires it or the current lane is physically
blocked. If the lane is blocked, take the shortest safe deviation and return to
a travel lane as soon as possible.
Keep right: when the intended lane is ambiguous, prefer the rightmost travel
lane in the flight direction.
When a measured lane reference is supplied, use its lateral offset, heading
error and lane-centre lookahead as the primary lateral guide. The reference is
measured at the planning anchor where your new actions begin (the end of the
pending queue), so its heading error already includes every pending action.
Never apply the pending turn a second time.
Reduce the lateral offset and heading error smoothly across the five actions;
do not overcorrect with a single large turn.

Step-length default (open lane):
When the lane ahead is clear and the UAV is roughly centred and aligned -
lateral offset within about 0.5 m, heading error within about 20 deg, and at
least about 1 m to the nearer lane boundary - use normal forward steps of
4-5 m for all five actions, giving a reach of about 20-25 m. At the cruising
speed one step is roughly one second of flight; a 1-2 m step lasts a fraction
of a second, drains the pending waypoint queue, and forces the UAV to stop and
wait for the next plan. Short first steps are only the tight-space exception
below, never the default on an open lane.

Lane-centring steering procedure:
1. Add up the five dx values to get the plan's forward reach R.
2. The plan should end near the lane-centre lookahead point nearest R and
   aligned with its cumulative lane yaw. Call that cumulative lane yaw T. The
   sum of the five dyaw values is the final heading change, so in normal
   steering it must end within about 5 deg of T. The near-point bearing is
   only the early aim direction and must be unwound before the plan ends.
3. Each action translates along the current heading before its dyaw_deg is
   applied, so a long first step while the UAV is angled away from the lane
   centre drifts sideways: the sideways drift of a step is about
   dx * sin(heading error). Keep that drift smaller than the distance to the
   nearer lane boundary. When a lane boundary is within about 0.5 m, or the
   heading error is larger than about 20 deg, start with one or two short
   pivot-like steps (about 1 m) that remove most of the heading error, then
   lengthen the later steps. Do not translate far while the UAV is still
   angled across the lane.
4. Aim the early actions at the near lane-centre point (about 5-6 m ahead).
   Its bearing is the direction to the lane centre and already includes the
   lateral offset. When that bearing differs from the near point's cumulative
   lane yaw by more than about 3 deg, the first one or two actions must turn
   toward the bearing, and the last one or two actions must unwind back toward
   T. A plan whose total equals the near-point bearing instead of T leaves the
   UAV angled across the lane: it keeps drifting sideways after the plan ends,
   and the next plan must reverse the turn, producing a left-right weave.
5. Matching only the heading error or the tangent yaw keeps the UAV parallel
   but off-centre; the offset then grows until the UAV crosses the boundary.
Worked example, tight-space exception (applies only when a lane boundary is
very close): anchor offset +1.3 m right of centre with the right boundary
only 0.42 m away, near 6 m point bearing -18.8 deg (near tangent -7 deg), and
the 22 m lane-centre tangent about -12 deg. A correct plan turns left early
and settles back: forward steps 1.5, 2.5, 5, 5, 5 m (reach 19 m) with
dyaw_deg -6, -6, -4, +2, +2 (total -12, close to the 22 m tangent). The short
first steps stop the right-angled UAV from drifting into the right boundary
before the left turn takes effect; every predicted pose stays inside the
lane. A uniform -1.4 deg per action (total -7) never reduces the 1.3 m offset
and drives the UAV into the right boundary. A full 5 m first step with -7 deg
also drifts 0.5 m right before turning and clips that boundary.
This plan shortens the first steps only because the right boundary is 0.42 m
away; with about 1.8 m on both sides the same anchor calls for normal 4-5 m
steps.
Second worked example, large-heading-error recovery (exception, heading error
over about 20 deg): anchor offset -0.95 m left of centre with 0.80 m to the
left boundary, heading error +46 deg (the lane points 46 deg right of the
UAV), near tangent +45 deg. A 5 m first step drifts
5 * sin(46 deg) = 3.6 m left and leaves the lane immediately, even though the
plan turns +10 deg. A correct plan pivots first: forward steps 0.5, 1.5, 3,
4, 4 m with dyaw_deg +20, +18, +14, +10, +6; the first step drifts only 0.4 m
and the cumulative +68 deg brings the UAV back across the centreline. Never
start a large-heading-error recovery with a long step; this exception does
not apply when the UAV is already aligned.
Open-lane worked example (the normal case): anchor offset +0.1 m right of
centre, heading error +1 deg, about 1.8 m to both lane boundaries, near
bearing +1.6 deg, and cumulative lane yaw at the reach about +1 deg. A
correct plan uses forward steps 5, 5, 5, 5, 5 m (reach 25 m) with dyaw_deg
0, 0, 0, 0, +1. Copying the short 1.5 m first step of the exception examples
into this situation is wrong and stalls the queue.
Weave counterexample: anchor offset +1.7 m right of centre with the right
boundary only 0.05 m away, near bearing -20.4 deg, near tangent -3.5 deg, and
the cumulative lane yaw at 20-30 m only -7.5 deg. A plan of -8, -8, -4, 0, 0
(total -20) keeps the near bearing as the total; the UAV turns across the
lane, the next plan reverses by about +18 deg, and the UAV weaves left-right.
A correct plan still uses short early steps to leave the right boundary, but
unwinds before the reach: forward steps 1.0, 1.5, 4, 5, 5 m with dyaw_deg
-5, -5, -3, +3, +2 (total -8, close to the -7.5 deg reach tangent).
A clear bearing window is useful only inside the current lane. Never leave the
lane to follow a wider gap.

Mandatory aim check before output:
The lane reference uses the same sign convention as dyaw_deg: positive is to
the RIGHT of the UAV, negative is to the LEFT.
This reference is measured at the anchor of your new actions; pending actions
are already included, so never add their turn again.
When the lateral offset is more than about 0.3 m or the near lane-centre
bearing is clearly nonzero, the first one or two actions must turn toward that
bearing; it is the early aim direction.
The total turn is a different quantity: add up the five dyaw_deg values. In
normal steering (anchor heading error within about 20 deg), the total must be
within about 5 deg of the cumulative lane yaw at the lookahead nearest the
plan reach (T). The later actions must unwind so the final heading matches T.
A total that matches the near-point bearing instead of T points the UAV
across the lane: the next plan must reverse it and the UAV weaves left-right.
Example: a near-point bearing of +38.8 deg means the lane centre is 38.8 deg
to the right, so the early actions must turn right. If T is +12 deg the total
should be near +12, not +38.8. If T is -3 deg (the lane curves back left),
the early actions still turn right and the later actions unwind through zero
to a slightly negative total.
When the anchor heading error exceeds about 20 deg, the total may exceed T to
recover the heading, but the last actions must still unwind toward the lane
and no predicted pose may point across the far boundary.
Before returning the JSON list, add up the five dyaw_deg values and confirm
the total is within about 5 deg of T in normal steering, and that the first
actions move toward the near-point bearing.

Measured free-space rules:
A measured metric free-space profile is supplied for every observation.
Treat its distances and bearings as measured values, not visual estimates.
Keep the entire planned path clear of every listed obstacle, and prefer the
clear bearing window inside the current lane.
When a listed distance is shorter than the planned forward step, shorten the
step or begin turning earlier.
A clear sector does not prove that unseen space is obstacle-free.

Increment rules:
dyaw_deg is incremental relative to the preceding waypoint, so turns
accumulate across the five actions. For example, five actions with -8 degrees
each produce a total turn of -40 degrees. Translations accumulate in the same
way along the predicted poses.
Choose enough accumulated turn to keep every predicted pose clear of the
listed obstacles and inside the current lane, using the measured bearing of the
lane centreline and of the open corridor within it.
"""


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
        "Lane reference (measured from the simulator HD map at the planning "
        "anchor for these new actions - the predicted pose after the pending "
        "queue, or the current pose when nothing is pending):"
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