"""Completion checker and trace construction from the shared configuration."""

from __future__ import annotations

from ...adapters.diagnostics import VisualGoalTrace
from ...adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from ...adapters.task_completion.prompts import VisualGoalCompletionPrompt
from ...config import AppConfig
from . import planners


def build_completion_checker(config: AppConfig):
    settings = config.task_completion
    if not settings.url or not settings.model:
        raise ValueError(
            "visual goal requires task_completion.url/model in the config"
        )
    key = planners.api_key(settings, 'task_completion')
    return OpenAICompatibleTaskCompletionChecker(
        settings.url,
        settings.model,
        key,
        settings.timeout_s,
        prompt_builder=VisualGoalCompletionPrompt(),
        max_completion_tokens=settings.max_completion_tokens,
        reasoning_effort=settings.reasoning_effort,
        request_format=settings.request_format,
    )


def build_trace_factory(config: AppConfig, *, detector=None, planner=None):
    """Create one filesystem trace directory per visual-goal task."""
    settings = config.visual_goal_navigation
    limits = settings.limits

    def _model(component):
        return getattr(getattr(component, "transport", None), "model", None)

    def trace_factory():
        trace = VisualGoalTrace(settings.output_root)
        trace.event(
            "runtime_configuration",
            goal_model=_model(detector),
            trajectory_model=_model(planner),
            waypoint_fields=("dx", "dy", "dz", "dyaw_deg"),
            goal_protocol="normalized_bbox_surface_points_v2",
            target_point_source="first_anchor_bounded_target_state",
            execution_policy="direct_model_trajectory",
            motion_limits=config.motion,
            speed_mps=config.motion.max_speed_mps,
            continuous_tracking=config.continuous,
            depth_range=(config.depth.min_m, config.depth.max_m),
            depth_foreground_filter=settings.depth_foreground_filter,
            depth_horizontal_fov_deg=config.camera.hfov_deg,
            execution_mode=(
                "continuous_world_queue"
                if config.sliding_window.enabled
                else "stopped_segments"
            ),
            planning_interval_s=config.sliding_window.planning_interval_s,
            max_buffer_points=config.sliding_window.max_buffer_points,
            target_refresh_interval_s=limits.target_refresh_interval_s,
            target_position_update_radius_m=limits.target_position_update_radius_m,
            camera_mount_position_body_m=config.camera.offset_m,
            target_depth_sampling_mode="direct_pixel",
        )
        return trace

    return trace_factory
