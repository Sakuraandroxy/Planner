"""Completion-only messages; never used by the trajectory planner."""
import json

from planner.adapters.model_visual_inputs import aligned_depth_image
from planner.adapters.task_completion.prompt import CompletionMessages
from planner.workflows.visual_goal_navigation import require_visual_completion_evidence
from planner.domain import depth_display_description
from planner.domain import VisualGoalCompletionInput

from .render import load_fragment, render


def _binding_evidence(request: VisualGoalCompletionInput) -> str:
    """Stage binding and detector claims are labelled; neither is ground truth."""
    target_state = request.target_state
    lines = [
        load_fragment(
            "binding_evidence_bound" if target_state else "binding_evidence_unbound"
        )
    ]
    evidence = request.target_observation
    observation = request.observation
    if (evidence is None or evidence.stage_id != request.stage_id
            or evidence.timestamp_ns > observation.timestamp_ns
            or evidence.captured_monotonic > observation.captured_monotonic):
        lines.append(load_fragment("binding_evidence_missing"))
        return "\n".join(lines)
    encode = lambda value: json.dumps(value, ensure_ascii=False, allow_nan=False)
    goal = evidence.goal
    lines.extend((
        render(
            load_fragment("binding_evidence_source_id"),
            capture_id=evidence.capture_id,
            age_s=f"{observation.captured_monotonic-evidence.captured_monotonic:.3f}",
        ),
        render(
            load_fragment("binding_evidence_same_observation"),
            same_observation=str(evidence.capture_id == observation.capture_id).lower(),
        ),
        render(
            load_fragment("binding_evidence_detector_status"),
            status=goal.status.value,
            same_target=str(goal.same_target).lower(),
        ),
        render(
            load_fragment("binding_evidence_detector_description"),
            description=goal.description,
            association_source=goal.association_source,
        ),
        render(load_fragment("binding_evidence_detector_detail"), evidence=goal.evidence),
        render(load_fragment("binding_evidence_bbox"), bbox_json=encode(goal.bbox)),
    ))
    if goal.surface_points is not None:
        lines.append(render(
            load_fragment("binding_evidence_surface_points"),
            surface_points_json=encode(goal.surface_points),
        ))
    if evidence.position_update_rejection is not None:
        lines.append(render(
            load_fragment("binding_evidence_rejected_update"),
            rejection=evidence.position_update_rejection,
        ))
    else:
        lines.append(load_fragment("binding_evidence_accepted_update"))
    return "\n".join(lines)


class VisualGoalCompletionPrompt:
    @staticmethod
    def prepare(request: VisualGoalCompletionInput) -> CompletionMessages:
        require_visual_completion_evidence(request)
        images = ((request.observation.rgb, "JPEG"), (aligned_depth_image(request.observation), "PNG"))
        if request.reference_rgb is not None:
            images += ((request.reference_rgb, "JPEG"),)
        return CompletionMessages(VisualGoalCompletionPrompt.system_prompt(),
            VisualGoalCompletionPrompt.build_user_prompt(request), images)

    @staticmethod
    def system_prompt():
        return load_fragment("system")

    @staticmethod
    def build_user_prompt(request: VisualGoalCompletionInput):
        parameters, geometry, observation = request.parameters, request.geometry, request.observation
        pose = observation.pose
        radius_met = geometry.distance_m+request.uncertainty_margin_m <= request.rules.radius_m
        height_met = (request.height_tolerance_m is None or
                      abs(pose.z-request.initial_pose.z) <= request.height_tolerance_m)
        height_description = (load_fragment("height_description_unlimited")
            if request.height_tolerance_m is None else
            render(load_fragment("height_description_tolerance"),
                   height_tolerance_m=request.height_tolerance_m))
        reference_input = (load_fragment("reference_input")
                           if request.reference_rgb is not None else "")
        required = "\n".join("- " + item for item in request.rules.required_conditions)
        encode = lambda value: json.dumps(value, ensure_ascii=False, allow_nan=False)
        target_state = request.target_state
        source = render(
            load_fragment("source_bound" if target_state else "source_current"),
            position_source=geometry.position_source)
        direction_reference = load_fragment("direction_reference")
        reference_rule = load_fragment(
            "reference_rule_with_reference" if request.reference_rgb is not None
            else "reference_rule_without_reference")
        distance_description = load_fragment(
            "distance_description_bound" if target_state else "distance_description_current")
        local_checks = load_fragment(
            "local_checks_bound" if target_state else "local_checks_current")
        if target_state:
            position_evidence = render(
                load_fragment("position_evidence_bound"),
                anchor_world_point=encode(target_state.anchor_world_point),
                known_surfaces=encode(target_state.known_surfaces),
                capture_id=target_state.capture_id,
                age_s=f"{observation.captured_monotonic-target_state.captured_monotonic:.3f}",
            )
        else:
            position_evidence = render(
                load_fragment("position_evidence_current"),
                bbox_json=encode(request.goal.bbox))
        return render(
            load_fragment("user_prompt"),
            reference_input=reference_input,
            depth_protocol=depth_display_description(
                observation.depth_mapping, observation.depth_min_m, observation.depth_max_m),
            stage_id=request.stage_id,
            instruction=parameters.instruction,
            target_description=parameters.target_description,
            reference_description=request.reference_description,
            association_source=request.goal.association_source,
            position_evidence=position_evidence,
            target_world_point=encode(geometry.world_point),
            source=source,
            keep_target_in_view=str(parameters.keep_target_in_view).lower(),
            initial_pose_json=encode([request.initial_pose.x, request.initial_pose.y,
                                      request.initial_pose.z, request.initial_pose.yaw_deg]),
            direction_reference=direction_reference,
            binding_evidence=_binding_evidence(request),
            pause_radius_m=request.rules.effective_pause_radius_m,
            radius_m=request.rules.radius_m,
            uncertainty_margin_m=request.uncertainty_margin_m,
            height_description=height_description,
            pose_json=encode([pose.x, pose.y, pose.z]),
            yaw_deg=pose.yaw_deg,
            distance_description=distance_description,
            distance_m=geometry.distance_m,
            radius_met=str(radius_met).lower(),
            height_met=str(height_met).lower(),
            local_checks=local_checks,
            confirmation_count=request.confirmation_count,
            required_confirmation_count=request.required_confirmation_count,
            required_conditions=required,
            reference_rule=reference_rule,
        )
