"""Per-run policy state; no model/device dependencies."""
from dataclasses import dataclass, field
import math
import re

from planner.domain.pose import wrap_yaw_deg
from planner.domain.visual_goal import VisualGoalStatus
from planner.domain.visual_goal_limits import VisualGoalLimits


class VisualGoalFailure(Exception):
    def __init__(self, status: VisualGoalStatus, reason: str):
        super().__init__(reason)
        self.status = status


@dataclass
class VisualGoalExitPolicy:
    limits: VisualGoalLimits
    planning_calls: int = 0
    unreliable: int = 0
    anchor_failures: int = 0
    progress_distances: list[float] = field(default_factory=list)
    stalled_moves: int = 0

    def record_motion(self, previous, current):
        translated = math.dist((previous.x, previous.y, previous.z), (current.x, current.y, current.z))
        yaw = abs(wrap_yaw_deg(current.yaw_deg-previous.yaw_deg))
        self.stalled_moves = self.stalled_moves+1 if translated < 0.2 and yaw < 2.0 else 0
        if self.stalled_moves >= 2:
            raise VisualGoalFailure(VisualGoalStatus.STUCK, "two executions produced no measured movement")
        return translated

    def planning_call(self):
        if self.planning_calls >= self.limits.maximum_planning_rounds:
            raise VisualGoalFailure(VisualGoalStatus.TIMEOUT, "planning request budget exhausted")
        self.planning_calls += 1

    def unreliable_observation(self, ambiguous=False):
        self.unreliable += 1
        if self.unreliable >= self.limits.maximum_unreliable_observations:
            raise VisualGoalFailure(VisualGoalStatus.TARGET_AMBIGUOUS if ambiguous else
                                    VisualGoalStatus.PERCEPTION_UNRELIABLE, "target/depth evidence remains unreliable")

    def record_progress(self, distance):
        self.progress_distances.append(distance)
        window = self.progress_distances[-self.limits.progress_window_rounds:]
        if len(window) == self.limits.progress_window_rounds and window[0]-min(window[1:]) < self.limits.minimum_progress_m:
            raise VisualGoalFailure(VisualGoalStatus.STUCK, "approach produced no reliable distance progress")

    def anchor_valid(self, observation, actual, now):
        original = observation.vehicle_pose
        valid = (observation.captured_monotonic > 0 and
                 0 <= now-observation.captured_monotonic <= self.limits.maximum_observation_age_s and
                 math.dist((actual.x, actual.y, actual.z), (original.x, original.y, original.z)) <=
                 self.limits.maximum_anchor_translation_m and
                 abs(wrap_yaw_deg(actual.yaw_deg-original.yaw_deg)) <= self.limits.maximum_anchor_yaw_deg)
        # A fresh capture being stable must not erase repeated drift during
        # subsequent model inference. Reset only after an accepted execution.
        if not valid:
            self.anchor_failures += 1
        if self.anchor_failures >= self.limits.maximum_anchor_refresh_failures:
            raise VisualGoalFailure(VisualGoalStatus.PERCEPTION_UNRELIABLE, "observation anchor repeatedly stale/drifting")
        return valid


@dataclass
class SearchPolicy:
    limits: VisualGoalLimits
    episodes: int = 0
    total_moves: int = 0
    headings: list[float] = field(default_factory=list)
    active: bool = False

    def next_heading(self, pose, reference=None, direction_hint=""):
        if not self.limits.search_enabled:
            raise VisualGoalFailure(VisualGoalStatus.BLOCKED, "target search is disabled")
        if not self.headings:
            if self.active:
                raise VisualGoalFailure(VisualGoalStatus.TARGET_LOST if reference else
                                        VisualGoalStatus.TARGET_NOT_FOUND, "current search scan exhausted")
            if self.episodes >= self.limits.maximum_search_episodes:
                raise VisualGoalFailure(VisualGoalStatus.TARGET_LOST if reference else
                                        VisualGoalStatus.TARGET_NOT_FOUND, "search episodes exhausted")
            self.episodes += 1
            self.active = True
            origin = pose.yaw_deg
            sign = -1
            if re.search(r"\bright\b|右", direction_hint, re.IGNORECASE):
                sign = 1
            if reference is not None:
                point = reference.world_point
                desired = math.degrees(math.atan2(point[1]-pose.y, point[0]-pose.x))
                sign = 1 if wrap_yaw_deg(desired-origin) > 0 else -1
            steps = math.ceil(self.limits.search_half_span_deg/self.limits.search_step_deg)
            side = [min(i*self.limits.search_step_deg, self.limits.search_half_span_deg) for i in range(1, steps+1)]
            offsets = [sign*v for v in side]+[sign*v for v in reversed(side[:-1])]+[0]+[-sign*v for v in side]
            self.headings = [wrap_yaw_deg(origin+v) for v in offsets[:self.limits.maximum_moves_per_search]]
        if self.total_moves >= self.limits.maximum_search_moves:
            raise VisualGoalFailure(VisualGoalStatus.TARGET_LOST if reference else
                                    VisualGoalStatus.TARGET_NOT_FOUND, "search move budget exhausted")
        self.total_moves += 1
        return self.headings.pop(0)

    def found(self):
        self.headings.clear()
        self.active = False
