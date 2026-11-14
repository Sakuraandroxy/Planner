"""Above-only boolean completion; does not require a full-roof image."""
from dataclasses import asdict
import json
from planner.adapters.task_completion.prompt import CompletionMessages
from planner.domain.depth import depth_display_description
from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.adapters.task_completion.above_evidence import marked_above_image
from planner.capabilities.targeting.top_geometry import region_summary


class VisualGoalAboveCompletionPrompt:
    def prepare(self, request):
        obs = request.observation
        system = ("Determine whether this UAV has completed the specified above-target stage. "
                  "Return exactly true or false. No explanation or Markdown.")
        images = ((marked_above_image(request),"JPEG"),(aligned_depth_image(obs),"PNG"))
        if request.reference_rgb is not None:
            images += ((request.reference_rgb,"JPEG"),)
        user = "\n".join((
            "Task: "+request.parameters.instruction,"Target: "+request.parameters.target_description,
            "Image 1: current world-downward single-camera RGB. Magenta marks the measured UAV body vertical projection, when visible. Image center is the camera axis, not the UAV body projection.",
            "Image 2: corresponding depth visualization.",
            "Image 3, if present: earlier reference of this same target, not another current camera.",
            depth_display_description(obs.depth_mapping,obs.depth_min_m,obs.depth_max_m),
            "Current camera rotation to world: "+json.dumps(obs.rotation_camera_to_world),
            "Current UAV pose: "+json.dumps(asdict(obs.vehicle_pose)),
            "Observed roof polygons with holes and local planes fitted from sparse metric depth supports: "+json.dumps([region_summary(r) for r in request.top_regions]),
            "Body projection world point: "+json.dumps(request.body_projection_world),
            "Body projection normalized image point: "+json.dumps(request.body_projection_image),
            "Height rule: "+("minimum clearance; greater height is allowed" if request.parameters.height_above_m is None else "explicit height with local measurement tolerance"),
            f"Required height above local top: {request.height_above_m} m",
            f"Measured height above local top: {request.measured_height_m} m",
            f"Vertical projection inside target top, excluding holes: {request.projection_inside}",
            f"Height condition met: {request.height_condition_met}",
            f"Position hold confirmed: {request.hold_confirmed}",
            "Above is world-vertical, not image-up. A nearby facade alone does not establish arrival.",
            "Use measured world-coordinate inclusion and height as required arrival evidence, not image appearance alone.",
            "Do not require the whole building, its center, or a full roof view. Darkness, partial visibility, or cropping alone do not negate established geometry.",
            "A roof elsewhere in the image does not establish that the UAV body is above it.",
            "If clear current evidence places the marked body projection over ground, a different object, or an opening outside the specified target top, return false even if prior geometry said inside.",
            "Grass is not a universal rejection category: the specified target may itself have a vegetated roof.",
            "The camera now looks world-downward; match the target using current and stage-observed evidence.",
            "A target leaving a forward view alone does not disprove the measured above relationship.",
            "Do not use unexecuted actions as evidence. If a required condition is unmet return false.",
            "Only output true or false."))
        return CompletionMessages(system,user,images)
