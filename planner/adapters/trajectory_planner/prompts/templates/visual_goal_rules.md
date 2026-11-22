Static visual goal approach rules:
The objective is to enter arrival_radius_m around the target's estimated position and stop.
Any position inside that radius is acceptable; do not aim for its exact boundary.
Use distance_uncertainty_margin_m for the arrival estimate. Preserve the selected object's identity.
The target is a locally estimated surface sample, not the object's 3D center or ground truth.
All initially accepted valid surface points remain bound to the selected object.
Use the supplied surface point vectors as a set. The single target vector selects the nearest
known surface horizontally at the observation pose; another supplied surface can be nearer
after pending motion. Re-evaluate this geometry at the NEW action reference.
Several samples do not describe the full object boundary or unseen surfaces.
Its first accepted position anchors the static target. Missing or rejected detection keeps
the last accepted position; its source and age do not prove current visibility.
Use the supplied target vectors [forward,right,down] in meters in the horizontal body frame.
target_relative_to_observation_body_m describes the image-time pose;
target_relative_to_pending_tail_body_m describes the predicted final pending pose.
With pending, NEW actions start at that predicted pose. Without pending, they start at the image pose.
The NEW trajectory planning reference below selects the starting target vector and bearing.
Use these estimates rather than re-estimating target distance or transforming world coordinates.
RGB, depth and camera geometry still inform scene/path choices and actual framing.
Initial directional descriptions identify the target at stage start; choose the current direction
from the planning reference rather than preserving its initial bearing.
If keep_target_in_view=true, coordinate translation and yaw to keep the bound target in front view.
When the planning reference places the target on the left side, never output a positive
dyaw_deg; when it places the target on the right side, never output a negative dyaw_deg.
Straight motion with dyaw_deg = 0 and turns toward the target remain allowed.
Plan useful progress toward the target and reduce motion near the arrival radius.
Arrival distance is horizontal distance to the nearest known target surface. Vertical separation
does not enter that radius. Surface coordinates still inform scene geometry.
This ground vehicle cannot change height; vertical motion is unavailable and dz must remain 0.
maximum_vertical_offset_m is therefore not a motion-planning constraint.
Choose useful evidence-supported motion; forward motion is not mandatory.
Do not plan blind lateral or backward motion into unobserved space.
A measured metric free-space profile lists the nearest above-ground obstacle distance
per bearing sector in the body frame. Keep every planned step clear of those measured
distances: do not advance into a sector whose measured obstacle distance is shorter
than that step's forward reach, and keep lateral or reverse motion out of measured
blocked sectors.
The executor follows valid trajectory points in order, with radius pauses for completion checks.
Use the execution history and pending to determine what was actually driven.
Previous executed command prefix is not an exact measured driving path; use actual pose updates.
Do not output completion status: the independent completion checker evaluates it after local measurements.

Mandatory trajectory output format:
Return exactly one JSON array containing exactly 5 waypoint arrays.
Each waypoint array must contain exactly 4 finite JSON numbers:
[dx, dy, dz, dyaw_deg], with dz = 0 for this ground vehicle.

Enclose all 5 waypoint arrays in ONE outer array.
Return the entire result on one line, starting with [[ and ending with ]].
Do not return five separate arrays without the outer brackets.

Output only the final numerical trajectory.
Do not output scene descriptions, calculations, reasoning, action descriptions,
introductions, Markdown, code fences, comments, field names, or completion status.
Do not output placeholders, ellipses, strings, booleans, null, NaN, or Infinity.
