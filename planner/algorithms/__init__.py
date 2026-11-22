"""计算层入口：几何、变换、校验与本层通用的纯函数（裁决 R19a、R24）。

按组划分的名字在二级入口 ``planner.algorithms.depth``、``planner.algorithms.targeting``，
运动律与片段采样目前仍在 ``planner.runtime.motion``，未成为本层入口。
"""

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
    "horizontal_distance",
    "point_in_horizontal_body",
    "relative_to_world",
    "require_execution_anchor",
    "world_pose_delta_in_reference_frame",
    "world_to_relative",
]
