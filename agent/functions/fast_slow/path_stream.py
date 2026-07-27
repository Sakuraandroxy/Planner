"""Non-blocking AirSim path streaming for sliding-window execution."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence


def _point3(value: Sequence[float]) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _passed_target(
    current_pos: Sequence[float],
    anchor_pos: Sequence[float],
    target_pos: Sequence[float],
    pass_tolerance_m: float,
) -> bool:
    anchor = _point3(anchor_pos)
    target = _point3(target_pos)
    pos = _point3(current_pos)
    segment = [target[i] - anchor[i] for i in range(3)]
    seg_sq = sum(value * value for value in segment)
    if seg_sq <= 1e-9:
        return True
    projection = sum((pos[i] - anchor[i]) * segment[i] for i in range(3)) / seg_sq
    closest = [anchor[i] + projection * segment[i] for i in range(3)]
    return projection >= 1.0 and _distance(pos, closest) <= pass_tolerance_m


def reached_prefix_count(
    current_pos: Sequence[float],
    anchor_pos: Sequence[float],
    waypoints: Sequence[Sequence[float]],
    *,
    reach_tolerance_m: float,
    pass_tolerance_m: float,
) -> int:
    """Return how many leading waypoints are reached or geometrically passed."""
    pos = _point3(current_pos)
    anchor = _point3(anchor_pos)
    consumed = 0
    for waypoint in waypoints:
        target = _point3(waypoint)
        if _distance(pos, target) <= reach_tolerance_m:
            consumed += 1
            anchor = target
            continue

        segment = [target[i] - anchor[i] for i in range(3)]
        seg_sq = sum(value * value for value in segment)
        if seg_sq <= 1e-9:
            consumed += 1
            anchor = target
            continue
        from_anchor = [pos[i] - anchor[i] for i in range(3)]
        projection = sum(from_anchor[i] * segment[i] for i in range(3)) / seg_sq
        closest = [anchor[i] + projection * segment[i] for i in range(3)]
        cross_track = _distance(pos, closest)
        if projection >= 1.0 and cross_track <= pass_tolerance_m:
            consumed += 1
            anchor = target
            continue
        break
    # A synchronous image capture can block the Python control loop while
    # AirSim keeps flying. The next pose may therefore be several curved
    # segments ahead. Recover progress from the closest segment instead of
    # leaving the first missed waypoint at the queue head forever.
    best_segment = -1
    best_projection = 0.0
    best_distance = float("inf")
    anchor = _point3(anchor_pos)
    for index, waypoint in enumerate(waypoints):
        target = _point3(waypoint)
        segment = [target[i] - anchor[i] for i in range(3)]
        seg_sq = sum(value * value for value in segment)
        if seg_sq <= 1e-9:
            anchor = target
            continue
        raw_projection = sum((pos[i] - anchor[i]) * segment[i] for i in range(3)) / seg_sq
        projection = max(0.0, min(1.0, raw_projection))
        closest = [anchor[i] + projection * segment[i] for i in range(3)]
        distance = _distance(pos, closest)
        if distance < best_distance:
            best_segment = index
            best_projection = raw_projection
            best_distance = distance
        anchor = target

    if best_segment >= 0 and best_distance <= pass_tolerance_m:
        recovered = best_segment
        target = waypoints[best_segment]
        if best_projection >= 1.0 or _distance(pos, target) <= reach_tolerance_m:
            recovered += 1
        consumed = max(consumed, recovered)
    return consumed


@dataclass(frozen=True)
class PathStreamPoll:
    consumed: int = 0
    collided: bool = False


class ContinuousPathStream:
    """Keep one AirSim path active and rewrite it when the local tail grows."""

    def __init__(self, client, config=None):
        config = config or {}
        self.client = client
        self.reach_tolerance_m = float(config.get("PATH_REACH_TOLERANCE_M", 0.8))
        self.final_reach_tolerance_m = float(config.get("PATH_FINAL_REACH_TOLERANCE_M", 0.2))
        self.pass_tolerance_m = float(config.get("PATH_PASS_TOLERANCE_M", 1.5))
        self.min_reissue_interval_s = float(config.get("PATH_REISSUE_MIN_INTERVAL_S", 0.15))
        self.velocity_epsilon_mps = float(config.get("PATH_VELOCITY_EPSILON_MPS", 0.05))
        self.debug_logs = bool(config.get("DEBUG_LOGS", False))
        self._anchor: list[float] | None = None
        self._commanded_remaining: list[list[float]] = []
        self._collision_marker = None
        self._last_issue_at = 0.0
        self._commanded_velocity: float | None = None

    @property
    def active(self) -> bool:
        return bool(self._commanded_remaining)

    def reset(self) -> None:
        self._anchor = None
        self._commanded_remaining = []
        self._collision_marker = None
        self._last_issue_at = 0.0
        self._commanded_velocity = None

    def stop(self) -> None:
        if self.active:
            self.client.stop_waypoint_path()
        self.reset()

    def poll(self, queue_waypoints: Sequence[Sequence[float]], current_pos: Sequence[float]) -> PathStreamPoll:
        if not self.active or self._anchor is None:
            return PathStreamPoll()

        consumed = reached_prefix_count(
            current_pos,
            self._anchor,
            self._commanded_remaining,
            reach_tolerance_m=self.reach_tolerance_m,
            pass_tolerance_m=self.pass_tolerance_m,
        )
        # Do not declare the whole path exhausted while the vehicle is still
        # inside the broad intermediate-waypoint tolerance. AirSim would still
        # be approaching the endpoint, but the scheduler would start waiting
        # for Qwen too early.
        if consumed == len(self._commanded_remaining) and self._commanded_remaining:
            final_distance = _distance(current_pos, self._commanded_remaining[-1])
            final_anchor = (
                self._anchor
                if len(self._commanded_remaining) == 1
                else self._commanded_remaining[-2]
            )
            passed_final = _passed_target(
                current_pos,
                final_anchor,
                self._commanded_remaining[-1],
                self.pass_tolerance_m,
            )
            if final_distance > self.final_reach_tolerance_m and not passed_final:
                consumed -= 1
        if consumed > 0:
            self._anchor = list(self._commanded_remaining[consumed - 1])
            del self._commanded_remaining[:consumed]

        collided = self.client.has_collision_since(self._collision_marker)
        if collided:
            self.reset()
        elif not self._commanded_remaining:
            self.reset()
        return PathStreamPoll(consumed=consumed, collided=collided)

    def sync(
        self,
        queue_waypoints: Sequence[Sequence[float]],
        current_pos: Sequence[float],
        velocity: float,
    ) -> bool:
        desired = [_point3(waypoint) for waypoint in queue_waypoints]
        if not desired:
            return False
        velocity = float(velocity)
        velocity_changed = (
            self._commanded_velocity is None
            or abs(velocity - self._commanded_velocity) >= self.velocity_epsilon_mps
        )
        if self.active and desired == self._commanded_remaining and not velocity_changed:
            return False
        if self.active and time.perf_counter() - self._last_issue_at < self.min_reissue_interval_s:
            return False

        # Waypoints consumed from the front (desired is a suffix of
        # commanded_remaining).  The existing AirSim path is still valid;
        # reissuing would restart ForwardOnly with a new anchor, introducing
        # heading drift from repeated curved-path entry calculations.
        if self.active and len(desired) < len(self._commanded_remaining) and not velocity_changed:
            consumed = len(self._commanded_remaining) - len(desired)
            if desired == self._commanded_remaining[consumed:]:
                self._commanded_remaining = desired
                if consumed > 0 and len(self._commanded_remaining) > 0:
                    self._anchor = list(self._commanded_remaining[0])
                return False

        continuing = bool(
            self.active
            and desired[: len(self._commanded_remaining)] == self._commanded_remaining
        )
        mode = "extend" if continuing else "start"
        previous_anchor = list(self._anchor) if continuing and self._anchor is not None else None
        self._collision_marker = self.client.collision_marker()
        self.client.start_waypoint_path(desired, velocity=velocity)
        # Reissuing moveOnPathAsync is required when a suffix is appended or
        # the commanded speed changes.  The remaining prefix still belongs to
        # the original path, so its progress anchor must remain unchanged.
        # Resetting it to the current in-segment pose turns an old queue-head
        # waypoint into a new backwards segment.
        self._anchor = previous_anchor or _point3(current_pos)
        self._commanded_remaining = desired
        self._commanded_velocity = velocity
        self._last_issue_at = time.perf_counter()
        if self.debug_logs:
            print(f"  [PathStream] {mode} points={len(desired)}")
        return True
