"""Detector and geometry construction from the shared configuration."""

from __future__ import annotations

from ...adapters import MultimodalChatClient
from ...adapters.target_detection import (
    ManualTargetDetector,
    OpenAICompatibleTargetDetector,
)
from ...algorithms.depth import DepthGeometry
from planner.algorithms.depth import DepthForegroundFilter
from ...config import AppConfig
from planner.domain import NormalizedBoundingBox
from . import planners


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
        key = planners.api_key(settings, 'target_detection')
        transport = MultimodalChatClient(
            settings.url,
            settings.model,
            key,
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


def build_geometry(config: AppConfig) -> DepthGeometry:
    settings = config.visual_goal_navigation
    return DepthGeometry(
        minimum_m=config.target_detection.minimum_depth_m,
        verified=settings.geometry_verified,
        foreground_filter=DepthForegroundFilter(settings.depth_foreground_filter),
        depth_is_range=config.camera.depth_is_range,
    )
