"""Bounded path preview and latched yaw alignment; never edits queue endpoints."""
import math

from planner.domain.motion import ContinuousReference


def xyz(pose):
    return pose.x, pose.y, pose.z


def segment_length(item):
    return math.dist(xyz(item.start), xyz(item.end))


def is_rotation(item):
    return segment_length(item) <= 1e-9 and abs(item.yaw_delta_deg) > 1e-9


def distance_to_segment(point, start, end):
    delta = tuple(b-a for a, b in zip(start, end))
    length_sq = sum(x*x for x in delta)
    fraction = max(0., min(1., sum((p-a)*d for p, a, d in zip(point, start, delta))/length_sq)) if length_sq else 0.
    return math.dist(point, tuple(a+fraction*d for a, d in zip(start, delta)))


class ContinuousPathReference:
    def __init__(self, settings, limits):
        self.settings, self.limits = settings, limits
        self.reset()

    def reset(self):
        self.head_id, self.progress, self.aligning = None, 0., False

    def sample(self, items, pose, yaw_error, speed):
        head = items[0]
        if head.waypoint_id != self.head_id:
            self.head_id, self.progress, self.aligning = head.waypoint_id, 0., False
        a, b, p = xyz(head.start), xyz(head.end), xyz(pose)
        length = segment_length(head)
        if length > 1e-9:
            projected = sum((v-s)*(e-s) for v, s, e in zip(p, a, b))/length
            self.progress = max(self.progress, min(length, max(0., projected)))
        distance = math.dist(p, b)
        if (length-self.progress <= self.limits.position_tolerance_m and
                abs(yaw_error) > self.limits.yaw_tolerance_deg):
            self.aligning = True
        # Once aligning, drifting out of the position ball must not restore
        # full cruise speed and start the old brake/accelerate cycle again.
        target = b
        if length > 1e-9 and not self.aligning:
            lookahead = min(self.settings.max_lookahead_m,
                max(self.settings.min_lookahead_m, speed*self.settings.lookahead_time_s))
            progress = min(length, self.progress+lookahead)
            target = tuple(s+(e-s)*progress/length for s, e in zip(a, b))
            remaining = lookahead-(progress-self.progress)
            if abs(yaw_error) <= self.limits.yaw_tolerance_deg:
                for item in items[1:]:
                    next_length = segment_length(item)
                    if is_rotation(item) or remaining <= 0:
                        break  # Never preview through a pure rotation barrier.
                    if next_length <= 1e-9:
                        continue  # Zero-motion, zero-yaw padding is not a dwell.
                    fraction = min(1., remaining/next_length)
                    candidate = tuple(s+(e-s)*fraction for s, e in zip(xyz(item.start), xyz(item.end)))
                    if distance_to_segment(candidate, a, b) > self.limits.position_tolerance_m*.5:
                        # Limit corner cutting without moving an authorized
                        # endpoint or changing the pending continuation anchor.
                        low, high = 0., fraction
                        for _ in range(16):
                            mid = (low+high)*.5
                            probe = tuple(s+(e-s)*mid for s, e in zip(xyz(item.start), xyz(item.end)))
                            if distance_to_segment(probe, a, b) <= self.limits.position_tolerance_m*.5:
                                low = mid
                            else:
                                high = mid
                        target = tuple(s+(e-s)*low for s, e in zip(xyz(item.start), xyz(item.end)))
                        break
                    target = candidate
                    remaining -= next_length*fraction
        return ContinuousReference(target, distance, max(0., length-self.progress), length,
                                   self.progress, self.aligning, length <= 1e-9)
