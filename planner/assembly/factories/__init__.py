"""Build detectors, planners and geometry from the shared configuration.

The CLI runner and the HTTP server both use these helpers so a detector or
planner mode always resolves to the same adapter.
"""

from __future__ import annotations

from collections.abc import Collection

from ...adapters.diagnostics import VisualGoalTrace
from ...adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from ...adapters.task_completion.prompts import VisualGoalCompletionPrompt
from ...adapters.task_parser import OpenAITaskParser
from ...adapters.task_parser.tasks import builtin_task_codecs
from ...config import AppConfig
from planner.domain import TaskKind
from . import planners
from .detectors import build_detector, build_geometry
from .planners import api_key, build_free_navigation_planner, build_planner


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


def build_task_parser(
    config: AppConfig,
    *,
    kinds: Collection[TaskKind] | None = None,
    aerial: bool = False,
) -> OpenAITaskParser | None:
    """Build the strict mission parser; no endpoint means tasks are rejected.

    ``kinds`` restricts the advertised codecs to workflow kinds this session
    actually registered; ``None`` keeps every built-in kind. ``aerial`` selects
    the reference UAV prompt series for the drone arm.
    """
    settings = config.task_parser
    if not settings.url or not settings.model:
        return None
    key = planners.api_key(settings, 'task_parser')
    codecs = builtin_task_codecs(aerial=aerial)
    if kinds is not None:
        allowed = set(kinds)
        codecs = tuple(codec for codec in codecs if codec.kind in allowed)
    return OpenAITaskParser(
        settings.url,
        settings.model,
        key,
        settings.timeout_s,
        supported_kinds={codec.kind for codec in codecs},
        codecs=codecs,
        max_completion_tokens=settings.max_completion_tokens,
        reasoning_effort=settings.reasoning_effort,
        request_format=settings.request_format,
        aerial=aerial,
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


__all__ = [
    "api_key",
    "build_completion_checker",
    "build_detector",
    "build_free_navigation_planner",
    "build_geometry",
    "build_planner",
    "build_task_parser",
    "build_trace_factory",
]
