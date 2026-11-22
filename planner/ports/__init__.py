"""Ports are structural interfaces: simulator adapters need only match them.

入口只导出有跨层调用点的名字：跨层 import 一律指向这里，只有本层在用的名字留深层路径。
"""

from .capabilities import (
    AppearanceModelControl,
    FramePublisher,
    ReturnHome,
    StreamingViews,
    WeatherControl,
)
from .goal_observer import GoalObserver
from .goal_source import StreamGoalSource
from .observation_source import (
    ControlledObservationSource,
    MovingObservationSource,
    ObservationSource,
)
from .position_hold_vehicle import PositionHoldVehicle
from .streaming_vehicle import StreamingVehicle
from .target_detector import TargetDetector
from .task_completion import TaskCompletionChecker
from .task_parser import TaskParser
from .task_trace import TaskTrace
from .trajectory_planner import TrajectoryPlanner
from .vehicle import Vehicle

__all__ = [
    "AppearanceModelControl",
    "ControlledObservationSource",
    "FramePublisher",
    "GoalObserver",
    "MovingObservationSource",
    "ObservationSource",
    "PositionHoldVehicle",
    "ReturnHome",
    "StreamGoalSource",
    "StreamingVehicle",
    "StreamingViews",
    "TargetDetector",
    "TaskCompletionChecker",
    "TaskParser",
    "TaskTrace",
    "TrajectoryPlanner",
    "Vehicle",
    "WeatherControl",
]
