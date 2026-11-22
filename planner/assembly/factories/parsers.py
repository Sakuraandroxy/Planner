"""Task parser construction from the shared configuration."""

from __future__ import annotations

from collections.abc import Collection

from ...adapters.task_parser import OpenAITaskParser
from ...adapters.task_parser.tasks import builtin_task_codecs
from ...config import AppConfig
from planner.domain import TaskKind
from . import planners


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
