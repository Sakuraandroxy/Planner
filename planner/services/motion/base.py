from typing import Protocol

from planner.domain.trajectory import MotionTrajectory, WorldTrajectory

#凡是要充当 MotionPlanner 的对象，都应提供一个 create_motion() 方法，接收 WorldTrajectory，返回 MotionTrajectory
#此过程中都是使用世界坐标表示
class MotionPlanner(Protocol):
    def create_motion(self, trajectory: WorldTrajectory) -> MotionTrajectory: ...

