from planner.domain.pose import WorldPose


class FakeVehicle:
    def __init__(self, fail: bool = False):
        self.pose = WorldPose(0, 0, 0, 0)
        self.fail = fail
        self.segments = []
        self.cancelled = False
        self.hovered = False

    def current_pose(self):
        return self.pose

    def execute_segment(self, segment):
        if self.fail:
            raise RuntimeError("execution failed")
        self.segments.append(segment)
        self.pose = segment.end

    def cancel(self):
        self.cancelled = True

    def hover(self):
        self.hovered = True

    def collision_state(self):
        return False

