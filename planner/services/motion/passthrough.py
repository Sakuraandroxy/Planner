from planner.domain.trajectory import MotionSegment, MotionTrajectory, WorldTrajectory


class PassThroughMotionPlanner:
    def __init__(self, speed_mps: float):
        self.speed_mps = speed_mps

    def create_motion(self, trajectory: WorldTrajectory) -> MotionTrajectory:
        previous = trajectory.start
        segments = []
        for pose in trajectory.poses:
            segments.append(MotionSegment(previous, pose, target_speed_mps=self.speed_mps))
            previous = pose
        return MotionTrajectory(tuple(segments))

