"""Per-run policy state; no model/device dependencies."""
import math
import re
from dataclasses import dataclass, field

from planner.domain import wrap_yaw_deg
from planner.domain import VisualGoalStatus
from planner.domain import VisualGoalLimits


class VisualGoalFailure(Exception):
    def __init__(self, status: VisualGoalStatus, reason: str):
        super().__init__(reason)
        self.status = status


@dataclass
class VisualGoalExitPolicy:
    """一轮目标接近的退出判据与计数。

    停滞判据一：单次执行位移与偏航同时低于阈值算「没动」，连续两次判 ``STUCK``。
    出处：经验值，未标定（阈值写在 ``record_motion`` 里）。影响因素：段周期与
    ``min_duration_s``（执行越短位移越小）、位姿测量噪声、速度下限、目标距离。
    标定方法：同一目标重复执行，统计实际位移的分布，取能被速度下限解释的下界。
    """

    limits: VisualGoalLimits
    planning_calls: int = 0
    unreliable: int = 0
    anchor_failures: int = 0
    stalled_moves: int = 0

    def record_motion(self, previous, current) -> None:
        # 停滞判据一：0.2 m / 2.0 度算「没动」，连续 2 次判 STUCK。出处与影响因素见
        # 类注释；这三个数是经验值，没有实测记录。
        translated = math.dist((previous.x, previous.y, previous.z), (current.x, current.y, current.z))
        yaw = abs(wrap_yaw_deg(current.yaw_deg-previous.yaw_deg))
        self.stalled_moves = self.stalled_moves+1 if translated < 0.2 and yaw < 2.0 else 0
        if self.stalled_moves >= 2:
            raise VisualGoalFailure(VisualGoalStatus.STUCK, "two executions produced no measured movement")

    def planning_call(self):
        if self.planning_calls >= self.limits.maximum_planning_rounds:
            raise VisualGoalFailure(VisualGoalStatus.TIMEOUT, "planning request budget exhausted")
        self.planning_calls += 1

    def unreliable_observation(self, ambiguous=False):
        self.unreliable += 1
        if self.unreliable >= self.limits.maximum_unreliable_observations:
            raise VisualGoalFailure(VisualGoalStatus.TARGET_AMBIGUOUS if ambiguous else
                                    VisualGoalStatus.PERCEPTION_UNRELIABLE, "target/depth evidence remains unreliable")

    def anchor_violation(self, observation, actual, now):
        """Name the bound a stale anchor breaks, or return ``None``."""
        original = observation.pose
        if observation.captured_monotonic <= 0:
            return "observation has no capture time"
        age = now-observation.captured_monotonic
        if not 0 <= age <= self.limits.maximum_observation_age_s:
            return (f"age {age:.1f}s over {self.limits.maximum_observation_age_s:.0f}s")
        drift = math.dist((actual.x, actual.y, actual.z), (original.x, original.y, original.z))
        if drift > self.limits.maximum_anchor_translation_m:
            return (f"moved {drift:.2f}m over {self.limits.maximum_anchor_translation_m:.2f}m")
        turned = abs(wrap_yaw_deg(actual.yaw_deg-original.yaw_deg))
        if turned > self.limits.maximum_anchor_yaw_deg:
            return (f"turned {turned:.1f}deg over {self.limits.maximum_anchor_yaw_deg:.1f}deg")
        return None

    def anchor_valid(self, observation, actual, now):
        valid = self.anchor_violation(observation, actual, now) is None
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