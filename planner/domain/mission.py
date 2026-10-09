from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from .visual_goal import VisualGoalNavigationParameters
from .basic_motion import BasicMotionParameters

#枚举任务类型
class TaskKind(str, Enum):
    BASIC_MOTION = "basic_motion"
    VISUAL_GOAL_NAVIGATION = "visual_goal_navigation"


#长任务的一个任务阶段
@dataclass(frozen=True)
class MissionStage:
    stage_id: str
    kind: TaskKind
    parameters: BasicMotionParameters | VisualGoalNavigationParameters

#完整的任务计划
@dataclass(frozen=True)
class MissionPlan:
    protocol_version: str
    stages: tuple[MissionStage, ...]

