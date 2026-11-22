"""``Vehicle`` 端口的兑现：读侧转给引擎，段执行转给控制循环。

引擎只提供 ``current_pose``、``motion_capability``、``collision_state``、``cancel``、
``hover`` 与 ``stream_command``；段执行、节拍与停稳判定都在 ``ControlLoop`` 里。
"""

from __future__ import annotations

from planner.domain import MotionSegment, MotionTrajectory


class GuidedVehicle:
    def __init__(self, streaming, loop):
        self.streaming, self.loop = streaming, loop

    def current_pose(self):
        return self.streaming.current_pose()

    def motion_capability(self):
        return self.streaming.motion_capability()

    def collision_state(self):
        return self.streaming.collision_state()

    def cancel(self) -> None:
        return self.streaming.cancel()

    def hover(self) -> None:
        return self.streaming.hover()

    def stream_state(self):
        return self.streaming.stream_state()

    def stream_command(self, command, duration_s: float) -> None:
        return self.streaming.stream_command(command, duration_s)

    def execute_segment(
        self, segment: MotionSegment, *, budget=None, pose_checkpoint=None
    ) -> None:
        self.loop.run_segment(
            MotionTrajectory((segment,)),
            self.loop.segment_sink(),
            budget=budget,
            pose_checkpoint=pose_checkpoint,
        )
