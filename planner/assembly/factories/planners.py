"""Planner construction from the shared configuration."""

from __future__ import annotations

import logging
import os

from ...adapters.multimodal_chat import MultimodalChatClient
from ...adapters.trajectory_planner import QwenVLPlanner
from ...adapters.trajectory_planner.prompts import (
    FreeNavigationTrajectoryPrompt,
    VisualGoalTrajectoryPrompt,
)
from planner.algorithms.targeting import GeometricTargetPlanner
from ...config import AppConfig

logger = logging.getLogger(__name__)


def api_key(settings, label: str = "") -> str:
    """Prefer the key written in the config; fall back to its env var.

    A missing key used to surface only as a rejected instruction naming no cause, which
    sent an operator looking at prompts and schemas when the whole problem was the
    environment the service was started in. Saying it once here, while the session is
    still being assembled, points at the actual fix.
    """
    key = settings.api_key or (
        os.environ.get(settings.api_key_env, "") if settings.api_key_env else ""
    )
    if not key and label:
        logger.warning(
            "%s: %s is configured but no credential was found (api_key is empty and "
            "%s is unset); requests will be rejected unless this endpoint needs no "
            "auth", label, getattr(settings, "url", "endpoint"),
            "$" + (settings.api_key_env or "api_key_env"),
        )
    return key


def _build_trajectory_planner(config: AppConfig, prompt_builder) -> QwenVLPlanner:
    settings = config.trajectory_planner
    if not settings.url or not settings.model:
        raise ValueError(
            "vlm planning requires trajectory_planner.url/model in the config"
        )
    key = api_key(settings, 'trajectory_planner')
    transport = MultimodalChatClient(
        settings.url,
        settings.model,
        key,
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
