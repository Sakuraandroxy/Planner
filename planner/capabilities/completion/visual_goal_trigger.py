"""Task-local radius pause policy; no device calls, network calls or queue access."""
import math

from planner.domain.pose import wrap_yaw_deg
from planner.domain.visual_goal_completion import VisualGoalTriggerRules, VisualGoalTargetSnapshot


class VisualGoalRadiusTrigger:
    def __init__(self, rules: VisualGoalTriggerRules, maximum_age_s: float, clock):
        self.rules, self.maximum_age_s, self.clock = rules, maximum_age_s, clock
        self.snapshot: VisualGoalTargetSnapshot | None = None
        self.last_pause_pose = None
        self.last_reason: str | None = None
        self.armed = True

    def update(self, snapshot: VisualGoalTargetSnapshot):
        if self.snapshot is not None and self.snapshot.stage_id != snapshot.stage_id:
            raise ValueError("target snapshot belongs to another mission stage")
        self.snapshot = snapshot

    def checked_at(self, pose):
        """Latch the current check, including an initial observation already inside."""
        self.last_pause_pose = pose
        self.armed = False

    def should_pause(self, pose):
        snapshot = self.snapshot
        age = self.clock()-snapshot.captured_monotonic if snapshot else math.inf
        if snapshot is None or not 0 <= age <= self.maximum_age_s:
            self.last_reason = "target_reference_expired"
            return True
        distance = math.dist((pose.x, pose.y, pose.z), snapshot.position_world)
        if distance > self.rules.radius_m:
            self.armed = True
            return False
        previous = self.last_pause_pose
        progressed = previous is None or (
            math.dist((pose.x, pose.y, pose.z), (previous.x, previous.y, previous.z)) >= self.rules.rearm_translation_m or
            abs(wrap_yaw_deg(pose.yaw_deg-previous.yaw_deg)) >= self.rules.rearm_yaw_deg)
        if self.armed or progressed:
            self.checked_at(pose)
            self.last_reason = "completion_radius"
            return True
        return False

