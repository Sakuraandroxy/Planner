from planner.domain.pose import WorldPose, resolve_yaw_delta_deg
from planner.errors import MotionPaused


class FakeVehicle:
    def __init__(self, fail: bool = False):
        self.pose = WorldPose(0, 0, 0, 0)
        self.fail = fail
        self.segments = []
        self.cancelled = False
        self.hovered = False

    def current_pose(self):
        return self.pose

    def execute_segment(self, segment, *, budget=None, pose_checkpoint=None):
        if self.fail:
            raise RuntimeError("execution failed")
        self.segments.append(segment)
        self.pose = segment.end
        if pose_checkpoint is not None:
            yaw = segment.start.yaw_deg + resolve_yaw_delta_deg(
                segment.start.yaw_deg, segment.end.yaw_deg, segment.yaw_delta_deg)
            if pose_checkpoint(self.pose, yaw):
                raise MotionPaused(self.pose, yaw)

    def execute_basic_motion(self, start, target, yaw_delta_deg):
        if self.fail:
            raise RuntimeError("execution failed")
        self.pose = target

    def cancel(self):
        self.cancelled = True

    def hover(self):
        self.hovered = True

    def collision_state(self):
        return False

