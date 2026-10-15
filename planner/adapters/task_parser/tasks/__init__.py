"""Explicit built-in task codecs; importing this module registers nothing."""
from dataclasses import dataclass
from typing import Callable

from planner.domain.mission import TaskKind, TaskParameters
from .basic_motion import BASIC_MOTION_RULES, decode_basic_motion
from .visual_goal_navigation import VISUAL_GOAL_RULES, decode_visual_goal


@dataclass(frozen=True)
class TaskCodec:
    kind: TaskKind
    prompt_rules: str
    decode: Callable[[dict[str, object]], TaskParameters]

    def __post_init__(self):
        if not isinstance(self.kind, TaskKind) or not isinstance(self.prompt_rules, str) or not self.prompt_rules.strip():
            raise ValueError("task codec requires a finite kind and non-empty parser rules")
        if not callable(self.decode):
            raise ValueError("task codec requires a parameter decoder")


def builtin_task_codecs() -> tuple[TaskCodec, ...]:
    return (
        TaskCodec(TaskKind.BASIC_MOTION, BASIC_MOTION_RULES, decode_basic_motion),
        TaskCodec(TaskKind.VISUAL_GOAL_NAVIGATION, VISUAL_GOAL_RULES, decode_visual_goal),
    )
