"""Visual-goal navigation workflow."""

from planner.domain import TaskKind

from .completion import (
    VisualGoalCompletion,
    VisualGoalRadiusTrigger,
    VisualGoalVerification,
    require_visual_completion_evidence,
    retained_world_trajectory,
)
from .continuous import VisualGoalContinuous
from .policy import (
    ApproachSettings,
    ApproachStatus,
    ApproachTarget,
    TargetApproach,
    VisualGoalStageValidator,
    approach_settings,
)
from .targeting import BBoxGoalObserver, VisualGoalRefresh
from .termination import VisualGoalExitPolicy, VisualGoalFailure
from .workflow import VisualGoalNavigationWorkflow

#: Task kind this package implements.
TASK_KIND = TaskKind.VISUAL_GOAL_NAVIGATION
#: Port slots the task cannot run without; the assembly derives flags from them.
REQUIRED_PORTS: tuple[str, ...] = ("vehicle", "observation")

__all__ = [
    "REQUIRED_PORTS",
    "TASK_KIND",
    "ApproachSettings",
    "ApproachStatus",
    "ApproachTarget",
    "BBoxGoalObserver",
    "TargetApproach",
    "VisualGoalCompletion",
    "VisualGoalContinuous",
    "VisualGoalExitPolicy",
    "VisualGoalFailure",
    "VisualGoalNavigationWorkflow",
    "VisualGoalRadiusTrigger",
    "VisualGoalRefresh",
    "VisualGoalStageValidator",
    "VisualGoalVerification",
    "approach_settings",
    "require_visual_completion_evidence",
    "retained_world_trajectory",
]
