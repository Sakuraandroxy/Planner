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
