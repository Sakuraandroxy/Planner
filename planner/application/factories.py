"""Build detectors, planners and geometry from the shared configuration.

The CLI runner and the HTTP server both use these helpers so a detector or
planner mode always resolves to the same adapter.
"""

from __future__ import annotations

import os
from typing import Collection

from ..adapters.diagnostics import VisualGoalTrace
from ..adapters.multimodal_chat import MultimodalChatClient
from ..adapters.target_detection import (
    ManualTargetDetector,
    OpenAICompatibleTargetDetector,
)
from ..adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from ..adapters.task_completion.prompts import VisualGoalCompletionPrompt
from ..adapters.task_parser import OpenAITaskParser
from ..adapters.task_parser.tasks import builtin_task_codecs
from ..adapters.trajectory_planner import QwenVLPlanner
from ..adapters.trajectory_planner.prompts import (
    FreeNavigationTrajectoryPrompt,
    VisualGoalTrajectoryPrompt,
)
from ..capabilities.planning import GeometricTargetPlanner
from ..capabilities.targeting import DepthGeometry
from ..capabilities.targeting.depth_foreground import DepthForegroundFilter
from ..config import AppConfig
from ..domain.mission import TaskKind
from ..domain.target_detection import NormalizedBoundingBox


def _api_key(settings) -> str:
    """Prefer the key written in the config; fall back to its env var."""
    return settings.api_key or (
        os.environ.get(settings.api_key_env, "") if settings.api_key_env else ""
    )


def build_detector(
    config: AppConfig,
    mode: str,
    *,
    bbox: NormalizedBoundingBox
    | tuple[float, float, float, float]
    | None = None,
    surface_points: tuple[tuple[float, float], ...] = (),
):
    if mode == "manual":
        if bbox is None:
            raise ValueError(
                "manual detection requires a normalized xmin,ymin,xmax,ymax box"
            )
        box = (
            bbox
            if isinstance(bbox, NormalizedBoundingBox)
            else NormalizedBoundingBox(*bbox)
        )
        return ManualTargetDetector(box, surface_points=tuple(surface_points))
    if mode == "vlm":
        settings = config.target_detection
        if not settings.url or not settings.model:
            raise ValueError(
                "vlm detection requires target_detection.url/model in the config"
            )
        api_key = _api_key(settings)
        transport = MultimodalChatClient(
            settings.url,
            settings.model,
            api_key,
            settings.timeout_s,
            max_completion_tokens=settings.max_completion_tokens,
            reasoning_effort=settings.reasoning_effort,
            request_format=settings.request_format,
        )
        return OpenAICompatibleTargetDetector(
            transport,
            maximum_image_side=settings.max_image_side,
        )
    raise ValueError(f"unknown detector mode '{mode}'")


def _build_trajectory_planner(config: AppConfig, prompt_builder) -> QwenVLPlanner:
    settings = config.trajectory_planner
    if not settings.url or not settings.model:
        raise ValueError(
            "vlm planning requires trajectory_planner.url/model in the config"
        )
    api_key = _api_key(settings)
    transport = MultimodalChatClient(
        settings.url,
        settings.model,
        api_key,
        settings.timeout_s,
        max_completion_tokens=settings.max_tokens,
        reasoning_effort=settings.reasoning_effort,
        request_format=settings.request_format,
    )
    return QwenVLPlanner(
        transport,
        expected_points=settings.point_count,
        timeout_s=settings.timeout_s,
        prompt_builder=prompt_builder,
    )


def build_planner(config: AppConfig, mode: str, *, aerial: bool = False):
    if mode == "geometric":
        return GeometricTargetPlanner()
    if mode == "vlm":
        return _build_trajectory_planner(
            config,
            VisualGoalTrajectoryPrompt(aerial=aerial),
        )
    raise ValueError(f"unknown planner mode '{mode}'")


def build_free_navigation_planner(
    config: AppConfig, *, aerial: bool = False
) -> QwenVLPlanner | None:
    """Build the target-free road-following planner when configured."""
    settings = config.trajectory_planner
    if not settings.url or not settings.model:
        return None
    return _build_trajectory_planner(
        config,
        FreeNavigationTrajectoryPrompt(aerial=aerial),
    )


def build_completion_checker(config: AppConfig):
    settings = config.task_completion
    if not settings.url or not settings.model:
        raise ValueError(
            "visual goal requires task_completion.url/model in the config"
        )
    api_key = _api_key(settings)
    return OpenAICompatibleTaskCompletionChecker(
        settings.url,
        settings.model,
        api_key,
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
    api_key = _api_key(settings)
    codecs = builtin_task_codecs(aerial=aerial)
    if kinds is not None:
        allowed = set(kinds)
        codecs = tuple(codec for codec in codecs if codec.kind in allowed)
    return OpenAITaskParser(
        settings.url,
        settings.model,
        api_key,
        settings.timeout_s,
        supported_kinds={codec.kind for codec in codecs},
        codecs=codecs,
        max_completion_tokens=settings.max_completion_tokens,
        reasoning_effort=settings.reasoning_effort,
        request_format=settings.request_format,
        aerial=aerial,
    )


def build_geometry(config: AppConfig) -> DepthGeometry:
    settings = config.visual_goal_navigation
    return DepthGeometry(
        minimum_m=config.target_detection.minimum_depth_m,
        verified=settings.geometry_verified,
        foreground_filter=DepthForegroundFilter(settings.depth_foreground_filter),
        depth_is_range=config.camera.depth_is_range,
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