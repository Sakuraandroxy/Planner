"""Above-only prompt; existing front-view prompts and wire protocol are unchanged."""
from dataclasses import asdict
import json
import math

from planner.domain.depth import depth_display_description
from planner.services.trajectory_transform import relative_to_world, world_pose_delta_in_reference_frame
from planner.services.target_coordinates import point_in_horizontal_body
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.pose import wrap_yaw_deg
from planner.capabilities.targeting.top_geometry import region_summary, top_height
from planner.capabilities.targeting.above_planning_reference import continuation_reference


SYSTEM = """You plan short UAV trajectories for reaching above a specified static target.
The single camera may face forward, upward, obliquely or downward. Use its actual geometry.
Image-up is not world-up. Camera rotation is not body rotation or a requested UAV yaw turn.

Motion coordinate rules:
Each action is [dx,dy,dz,dyaw_deg]. Translation is in meters; yaw change is in degrees.
+dx means forward; -dx means backward. +dy means right; -dy means left.
+dz means DOWN; -dz means UP. Positive dyaw_deg turns right; negative turns left.
To ascend 2 meters, use dz=-2. To descend 2 meters, use dz=+2.
These two examples explain signs only, not a required step size.
A positive height above a roof does NOT mean a positive dz.
World z_next = z_previous + dz. Destination world z = roof world z - height above roof.
Return relative increments, not absolute world positions, altitude values or absolute yaw.
For every action, translate in the PRECEDING pose's horizontal body frame using its yaw,
THEN add dyaw_deg. Every later action uses its preceding newly predicted pose.
Camera pitch, roll and image orientation do not change these motion-coordinate conventions.

Pending continuation rules:
Apply supplied pending once from the image-time pose. NEW actions start at its final predicted
pose, or the current pose if pending is empty. Later NEW actions use their preceding predicted pose.
Images show the observation anchor, never the unseen viewpoint after pending.
Do not repeat scheduled motion, restart the task, or assume unseen space has been observed.

Above-target planning rules:
When finding the target, preserve the instruction's identity conditions and seek a useful view.
When revealing the top, neither a facade point nor the bbox upper edge is a measured roof height.
You plan ALL intentional translation, including ascent to reveal the top. The workflow changes
camera direction but does not insert ascent moves. Before top_detection_enabled, the detector
locates the target only; an empty top_regions in that mode does not mean a roof search failed.
When the bound bbox upper edge is above world-horizontal, favor useful ascent while keeping the
same target visible. A clipped upper edge does not give the target's full height. Use the actual
camera geometry; image-up and the image midpoint are not world-up or the world horizon.
Use connected, useful motion across the five actions. Do not end every action with an assumed
hover, or fill the batch with tiny repeated motions just to wait for detection. Account for ascent
already in pending; do not add the same remaining ascent five times. Choose motion from the scene,
not a fixed building height. Once useful roof evidence appears, update the continuation instead
of repeating ascent. With reported_top_visible but no usable geometry, improve the roof view;
do not treat missing reconstruction as proof that more ascent is always needed.
Do not descend toward the ground to reveal a roof. Looking down does not prove the body is above it.
top_observation belongs to its stated capture_id and pose; it is not a box in another current RGB.
When approaching above, use the supplied destination and observed surface regions. No building
center is required. Above means world-vertical. Reduce motion near the destination and account for
all pending motion. Surface points are samples, not a complete building or free-space map.
Roof regions are finite observed polygons with holes and local planes fitted from sparse depth
supports. The plane does not extend the roof beyond its outline. These are approximate observations,
not dense reconstructed surfaces. Newer observations take precedence over an older partial view.
last_executed_before_replan, when supplied, is confirmed earlier motion retained across queue
replacement, NOT pending. Its end pose is historical; use the current measured pose to continue.
Translation and yaw are independent; pure yaw and vertical movement are allowed.
Horizontal translation is optional. When the building is suitably positioned for observation,
you may ascend vertically while adjusting yaw to retain its view. Do not keep moving forward
merely because the previous batch moved forward. Do not keep turning left because the target
was initially described as front-left. Reduce the remaining yaw correction as the target aligns.
Actively turn the body to keep the specified target and useful roof material near the image
middle when the view is forward or oblique. Reduce clipping; do not default every dyaw to zero.
Turn during translation/ascent, or turn in place to improve observation. Centering improves
observation; it does not require reaching the building center. Recompute the remaining turn
after each predicted translation and yaw. Never repeat a full initial target-bearing correction
at all five points. Account for camera heading offset instead of compensating it twice.
With a world-downward camera, body yaw mainly rotates the image; use horizontal translation
to change which surface lies below the body. Use the actual camera pose to interpret pixels.
Unspecified height is a minimum top clearance; a higher useful observation position is allowed.
Explicit requested height is an arrival height, not a limit on temporary observation ascent.
Do not output camera controls or completion messages.
The bound target identity and measured world geometry persist when it leaves the image.
Do not select a different visible roof or reinterpret initial directional words as a new target.
geometric_continuation_reference starts AFTER pending and already accounts for changing yaw.
It is a coordinate reference toward the bound destination, not obstacle-aware flight approval.
Use it to avoid coordinate mistakes; adapt motion to the current scene and useful observation.
Recompute yaw incrementally; repeated large turns do not keep pointing at a fixed world target.
Return ONLY one JSON array with exactly 5 NEW arrays, exactly 4 finite numbers per array.
No Markdown, prose, objects, null, NaN, Infinity or absolute coordinates. Yaw increments are
within [-180,180]. Do not include pending.
"""


OUTPUT_GUIDANCE = """Final response:
Return only one complete JSON array of exactly 5 NEW waypoints.
Each waypoint contains exactly 4 finite numbers: [dx,dy,dz,dyaw_deg].
Start with [[ and end with ]]. Include the outer brackets.
Do not include pending, explanations, headings, Markdown or code fences.

Correct output example (format only, not the requested trajectory):
Choose values from the current task, images, geometry and pending actions.
[[0.5,0.1,-0.2,5.0],[0.6,0.1,-0.1,4.0],[0.5,0.0,0.0,2.0],[0.3,-0.1,0.0,-1.0],[0.1,0.0,0.0,0.0]]
"""


def _new_trajectory_reference(context, pose, tail, has_pending):
    start = (
        "Pending actions are already scheduled and will execute first.\n"
        "Start NEW actions at their predicted final pose, not at the image-time pose.\n"
        "Do not return, repeat or replace pending actions."
        if has_pending else
        "Pending is empty. Start NEW actions at the current observation pose."
    )
    lines = ["NEW trajectory planning reference:", start,
             "NEW action start pose [world x,y,z,yaw_deg]: " + json.dumps(
                 [tail.x,tail.y,tail.z,tail.yaw_deg]),
             f"Pending net vertical motion: {pose.z-tail.z:+.4f} m UP (negative means DOWN).",
             "All relative vectors below use [forward,right,NED down], in meters."]
    destination = context.get("destination_relative_to_pending_tail_body_m")
    if destination is not None:
        lines += ["Flight destination relative to the NEW action start: " + json.dumps(destination),
                  "Negative down-offset means the destination is ABOVE this start; positive means BELOW.",
                  "This is remaining displacement, not an action to repeat at all five waypoints.",
                  "Use the measured roof and destination to approach above the bound target.",
                  "Descent is allowed when appropriate; seeking a roof does not forbid later descent."]
    else:
        lines += [f"Current phase: {context['phase']}.",
                  "No roof flight destination has been measured yet.",
                  "Plan useful motion to reveal the top of the SAME bound building.",
                  "When the observed bbox upper edge is above world-horizontal, use upward motion",
                  "to improve the view. Upward motion requires NEGATIVE dz.",
                  "The bbox edge gives a viewing direction, not a measured roof height.",
                  "Its capture pose and ID belong to that observation, not the unseen pending-tail view.",
                  "Account for ascent already scheduled in pending before adding more ascent.",
                  "If pending already supplies a useful higher view, do not repeat the full ascent.",
                  "Do not descend toward the ground to reveal a roof."]
    if "target_relative_to_new_actions_start_body_m" in context:
        lines += ["Building observation reference relative to the NEW action start: " + json.dumps(
                      context["target_relative_to_new_actions_start_body_m"]),
                  "This reference helps locate and observe the building; it is NOT a flight destination.",
                  "A surface sample or average does not establish roof height or arrival altitude.",
                  "Building reference horizontal bearing: " + json.dumps(
                      context["target_bearing_from_new_actions_start_deg"]) + " degrees.",
                  "Positive bearing means right; negative means left. It is not a required yaw command."]
        correction = context["view_yaw_correction_at_new_actions_start_deg"]
        if correction is not None:
            lines += [f"Remaining view yaw correction at the NEW action start: {correction:+.2f} degrees.",
                      "This includes the current camera heading offset. Positive means RIGHT; negative means LEFT.",
                      "It is one remaining correction, not a turn to repeat at every waypoint."]
        else:
            lines += ["No horizontal view-yaw reference is defined for this geometry.",
                      "With a world-downward camera, use horizontal position to frame the target."]
    lines += ["Initial words such as front-left identify the original target, not a permanent motion direction.",
              "Turns accumulate: five -8 degree increments turn -40 degrees in total.",
              "Re-evaluate direction after each predicted translation and turn, accounting for camera heading.",
              "Images show the observation anchor, not the future view after pending."]
    return "\n".join(lines)


class VisualGoalAboveTrajectoryPrompt:
    def __init__(self, *, speed_mps=2., planning_interval_s=5., max_step_m=5.,
                 max_yaw_step_deg=30., max_yaw_rate_deg_s=30.):
        self.reference_step_m = min(max_step_m,speed_mps*planning_interval_s/5)
        self.reference_yaw_step_deg = min(max_yaw_step_deg,max_yaw_rate_deg_s*planning_interval_s/5)
        self.speed_mps = speed_mps
        self.planning_interval_s = planning_interval_s

    def system_prompt(self):
        return SYSTEM

    def build_user_prompt(self, instruction, observation, max_points, *, current_guidance=None,
                         guidance_start=None, round_index=1, initial_pose=None, previous_trajectory=None,
                         previous_trajectory_start=None, pending_trajectory=None, task_context=None):
        if max_points != 5 or task_context is None:
            raise ValueError("above planning requires its typed context and exactly five new actions")
        pose = observation.vehicle_pose
        pending = pending_trajectory or RelativeTrajectory(())
        tail = relative_to_world(pending,pose).poses[-1] if pending.points else pose
        measured = world_pose_delta_in_reference_frame(initial_pose or pose,pose)
        camera = {"camera_id":observation.camera_id,"intrinsics":asdict(observation.intrinsics),
                  "camera_position_world":observation.camera_position_world,
                  "rotation_camera_to_world":observation.rotation_camera_to_world}
        context = {"phase":task_context.phase.value,"target_description":task_context.target_description,
                   "surface_points_world":task_context.surface_points_world,
                   "bound_surface_points_world":task_context.bound_surface_points_world,
                   "top_regions":[region_summary(r) for r in task_context.top_regions],
                   "destination":asdict(task_context.destination) if task_context.destination else None,
                   "height_above_m":task_context.height_above_m,"height_is_minimum":task_context.height_is_minimum,
                   "bound_target_visible_at_last_detection":task_context.target_visible,
                   "top_detection_enabled":task_context.top_detection_enabled,
                   "reported_top_visible":task_context.reported_top_visible,
                   "roof_geometry_available":bool(task_context.top_regions),
                   "top_observation":asdict(task_context.top_observation) if task_context.top_observation else None,
                   "last_executed_before_replan":asdict(task_context.last_executed_before_replan) if task_context.last_executed_before_replan else None}
        context["surface_points_relative_to_observation_body_m"] = [
            point_in_horizontal_body(p,pose) for p in task_context.surface_points_world]
        context["surface_points_relative_to_pending_tail_body_m"] = [
            point_in_horizontal_body(p,tail) for p in task_context.surface_points_world]
        if task_context.destination is not None:
            target = task_context.destination
            point = (target.x,target.y,target.z)
            context.update(destination_relative_to_observation_body_m=point_in_horizontal_body(point,pose),
                           destination_relative_to_pending_tail_body_m=point_in_horizontal_body(point,tail))
        identity_points = task_context.bound_surface_points_world or task_context.surface_points_world
        target = ((task_context.destination.x,task_context.destination.y,
                   top_height(task_context.top_regions,task_context.destination.x,task_context.destination.y)
                   if task_context.top_regions else task_context.destination.z)
                  if task_context.destination else
                  tuple(sum(p[i] for p in identity_points)/len(identity_points)
                        for i in range(3)) if identity_points else None)
        if target is None and task_context.destination is not None:
            target = (task_context.destination.x,task_context.destination.y,task_context.destination.z)
        forward = tuple(row[0] for row in observation.rotation_camera_to_world)
        context["observation_camera_forward_world"] = forward
        context["observation_camera_heading_offset_from_body_deg"] = (
            wrap_yaw_deg(math.degrees(math.atan2(forward[1],forward[0]))-pose.yaw_deg)
            if math.hypot(forward[0],forward[1]) > 1e-6 else None)
        if target is not None:
            vector = point_in_horizontal_body(target,tail)
            context["observation_target_world"] = target
            context["target_relative_to_new_actions_start_body_m"] = vector
            context["target_bearing_from_new_actions_start_deg"] = (
                math.degrees(math.atan2(vector[1],vector[0])) if math.hypot(vector[0],vector[1]) > 1e-6 else None)
            bearing = context["target_bearing_from_new_actions_start_deg"]
            offset = context["observation_camera_heading_offset_from_body_deg"]
            context["view_yaw_correction_at_new_actions_start_deg"] = (
                wrap_yaw_deg(bearing-offset) if bearing is not None and offset is not None else None)
        if task_context.destination is not None:
            context["geometric_continuation_reference"] = continuation_reference(tail,task_context.destination,
                target,context["observation_camera_heading_offset_from_body_deg"],
                step_m=self.reference_step_m,yaw_step_deg=self.reference_yaw_step_deg)
        history = None if previous_trajectory is None else [asdict(p) for p in previous_trajectory.points]
        horizon = self.planning_interval_s
        return "\n".join((
            "Visual inputs:","- Image 1: current RGB from the single camera identified below.",
            "- Image 2: aligned depth visualization from the same camera and observation.",
            "- "+depth_display_description(observation.depth_mapping,observation.depth_min_m,observation.depth_max_m),
            "\nOriginal navigation instruction for the current mission stage:",instruction,
            "\nCurrent guidance:","Not supplied.",
            "\nExecution context:",f"- Planning round: {round_index}",
            "- Measured stage-start net pose change: "+json.dumps(asdict(measured)),
            "- Recent confirmed executed actions, do not repeat: "+json.dumps(history),
            "- Execution-history reference start pose: "+json.dumps(asdict(previous_trajectory_start) if previous_trajectory_start else None),
            "\nCurrent UAV pose:",json.dumps(asdict(pose)),
            "\nCamera geometry (JSON): "+json.dumps(camera),
            "\nAbove-task context:\n"+json.dumps(context),
            "\nPending incremental pose waypoints: "+json.dumps([[p.dx,p.dy,p.dz,p.dyaw_deg] for p in pending.points]),
            "Pending is unexecuted at this observation anchor. Continue after its final predicted pose.",
            "\n"+_new_trajectory_reference(context,pose,tail,bool(pending.points)),
            "\nMotion scale reference:",
            f"Configured flight speed: {self.speed_mps:g} m/s.",
            f"Planning request interval: {self.planning_interval_s:g} s.",
            "Continue with the sliding window while revealing or approaching the roof; there is no periodic stop after each batch.",
            f"At this speed, the short-motion reference interval corresponds to about {self.speed_mps*horizon:g} meters of travel.",
            "Use this as a scale reference for a short continuation, not a fixed distance or duration.",
            "The request interval is not a duration assigned to each waypoint.",
            "Do not use five large jumps merely to reach a distant target in one response.",
            "\n"+OUTPUT_GUIDANCE.strip()))
