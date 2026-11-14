"""Explicit built-in task codecs; importing this module registers nothing."""
from dataclasses import dataclass
from typing import Callable

from planner.domain.mission import TaskKind, TaskParameters
from .basic_motion import (
    AERIAL_BASIC_MOTION_RULES,
    BASIC_MOTION_RULES,
    decode_basic_motion,
)
from .free_navigation import (
    AERIAL_FREE_NAVIGATION_RULES,
    FREE_NAVIGATION_RULES,
    decode_free_navigation,
)
from .visual_goal_navigation import (
    AERIAL_VISUAL_GOAL_RULES,
    VISUAL_GOAL_RULES,
    decode_visual_goal,
)


@dataclass(frozen=True)
class TaskCodec:
    kind: TaskKind
    prompt_rules: str
    decode: Callable[[dict[str, object]], TaskParameters]

    def __post_init__(self):
        if (
            not isinstance(self.kind, TaskKind)
            or not isinstance(self.prompt_rules, str)
            or not self.prompt_rules.strip()
        ):
            raise ValueError(
                "task codec requires a finite kind and non-empty parser rules"
            )
        if not callable(self.decode):
            raise ValueError("task codec requires a parameter decoder")


def builtin_task_codecs(*, aerial: bool = False) -> tuple[TaskCodec, ...]:
    """Decoders shared by both vehicles; only the prompt rules differ."""
    return (
        TaskCodec(
            TaskKind.BASIC_MOTION,
            AERIAL_BASIC_MOTION_RULES if aerial else BASIC_MOTION_RULES,
            decode_basic_motion,
        ),
        TaskCodec(
            TaskKind.VISUAL_GOAL_NAVIGATION,
            AERIAL_VISUAL_GOAL_RULES if aerial else VISUAL_GOAL_RULES,
            decode_visual_goal,
        ),
        TaskCodec(
            TaskKind.FREE_NAVIGATION,
            AERIAL_FREE_NAVIGATION_RULES
            if aerial
            else FREE_NAVIGATION_RULES,
            decode_free_navigation,
        ),
    )