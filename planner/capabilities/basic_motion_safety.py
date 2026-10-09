"""No perception claim: verify a whole straight segment inside a certified box."""
from planner.domain.basic_motion import ConfirmedFreeSpace, MotionSafetyDecision
from planner.domain.trajectory import MotionTrajectory


class BasicMotionSafety:
    def __init__(self, space: ConfirmedFreeSpace):
        self.space = space

    def check(self, motion: MotionTrajectory) -> MotionSafetyDecision:
        if not self.space.confirmed:
            return MotionSafetyDecision(False, "basic-motion free space has not been confirmed by the operator")
        if not motion.segments:
            return MotionSafetyDecision(False, "empty motion")
        radius = self.space.clearance_radius_m
        lo, hi = self.space.minimum_world_ned_m, self.space.maximum_world_ned_m
        # A box is convex; both endpoints inside the radius-eroded box certify
        # this straight centerline and its spherical swept-body envelope.
        for segment in motion.segments:
            for pose in (segment.start, segment.end):
                if any(v-radius < a or v+radius > b for v, a, b in zip((pose.x, pose.y, pose.z), lo, hi)):
                    return MotionSafetyDecision(False, "motion/body clearance exceeds operator-confirmed world space")
        return MotionSafetyDecision(True, "whole motion lies within operator-confirmed static free space")
