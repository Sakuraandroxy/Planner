"""计算层入口：几何、变换、校验与本层通用的纯函数（裁决 R19a、R24）。

按组划分的名字在二级入口 ``planner.algorithms.depth``、``planner.algorithms.targeting``、
``planner.algorithms.motion``，其余运动律与片段采样仍在 ``planner.runtime.motion``，尚未迁入。
"""

from .motion import bounded, corridor_default, passed_lateral_corridor, reached
from .sliding_window.scheduler import RefillScheduler
from .target_coordinates import horizontal_distance, point_in_horizontal_body
from .trajectory_transform import (
    relative_to_world,
    world_pose_delta_in_reference_frame,
    world_to_relative,
)
from .trajectory_validation import TrajectoryValidator, require_execution_anchor

__all__ = [
    "RefillScheduler",
    "TrajectoryValidator",
    "bounded",
    "corridor_default",
    "horizontal_distance",
    "passed_lateral_corridor",
    "point_in_horizontal_body",
    "reached",
    "relative_to_world",
    "require_execution_anchor",
    "world_pose_delta_in_reference_frame",
    "world_to_relative",
]
