"""Polyline following with continuous velocity and independent signed yaw."""
import math

from planner.domain.motion import ContinuousTrackingSettings
from .continuous_reference import ContinuousPathReference, segment_length, xyz
from .continuous_speed import braking_speed, planned_speed, corner_speed
from .command_smoothing import CommandSmoother


def reached(waypoint, pose, unwrapped_yaw, goal_yaw, position_tolerance, yaw_tolerance):
    a, b = waypoint.start, waypoint.end
    delta = (b.x-a.x, b.y-a.y, b.z-a.z)
    offset = (pose.x-a.x, pose.y-a.y, pose.z-a.z)
    norm_sq = sum(v*v for v in delta)
    distance = math.dist((pose.x, pose.y, pose.z), (b.x, b.y, b.z))
    passed = False
    if norm_sq > 1e-12:
        projection = sum(x*y for x, y in zip(offset, delta))/norm_sq
        lateral = math.sqrt(sum((x-projection*y)**2 for x, y in zip(offset, delta)))
        passed = projection >= 1 and lateral <= position_tolerance
    return ((distance <= position_tolerance or passed) and abs(goal_yaw-unwrapped_yaw) <= yaw_tolerance)


class ContinuousFollower:
    def __init__(self, speed_mps, limits, settings=ContinuousTrackingSettings()):
        if isinstance(speed_mps, bool) or not math.isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError("continuous speed must be finite and positive")
        self.speed, self.limits = speed_mps, limits
        self.settings = settings
        self.translation = CommandSmoother(limits.max_acceleration_mps2, settings.max_jerk_mps3)
        self.rotation = CommandSmoother(limits.max_yaw_acceleration_deg_s2, settings.max_yaw_jerk_deg_s3)
        self.path = ContinuousPathReference(settings, limits)
        self.reference = None
        self.previous_waypoint, self.entry_speed = None, self.speed

    @property
    def velocity(self):
        return self.translation.velocity

    @property
    def yaw_rate(self):
        return self.rotation.velocity[0]

    def reset(self, velocity=(0.0, 0.0, 0.0)):
        self.translation.reset(velocity)
        self.rotation.reset((0.,))
        self.path.reset()
        self.reference = None
        self.previous_waypoint, self.entry_speed = None, self.speed

    def sample(self, items, pose, yaw, goal_yaw, dt):
        limits, settings = self.limits, self.settings
        head = items[0]
        previous = self.previous_waypoint
        if previous is not None and previous.waypoint_id != head.waypoint_id:
            self.entry_speed = self.speed
            if min(segment_length(previous), segment_length(head)) > 1e-9:
                first = tuple(b-a for a,b in zip(xyz(previous.start),xyz(previous.end)))
                second = tuple(b-a for a,b in zip(xyz(head.start),xyz(head.end)))
                cosine = sum(a*b for a,b in zip(first,second))/(math.hypot(*first)*math.hypot(*second))
                if abs(cosine) < 1-1e-9:
                    self.entry_speed = corner_speed(previous,head,limits,settings,self.speed,
                                                    2/self.translation.bandwidth)
        self.previous_waypoint = head
        yaw_error = goal_yaw-yaw  # Unwrapped, preserving signed half/full turns.
        reference = self.path.sample(items, pose, yaw_error, math.hypot(*self.velocity))
        self.reference = reference
        error = tuple(v-p for v, p in zip(reference.position, (pose.x, pose.y, pose.z)))
        norm = math.hypot(*error)
        speed = planned_speed(items, reference, limits, settings, self.speed, self.translation)
        if reference.progress_m < min(settings.min_lookahead_m, reference.segment_length_m*.5):
            # Carry the cornering speed through the beginning of the outgoing
            # segment; otherwise head consumption immediately requests cruise
            # while residual incoming velocity still carries us past the turn.
            speed = min(speed, self.entry_speed)
        if reference.aligning or reference.pure_rotation or reference.remaining_head_m <= 1e-9:
            # Damped position hold/correction, latched across the old 0.3m
            # discontinuity. No full-speed chase of an endpoint behind us.
            speed = min(self.speed, limits.position_gain*norm,
                braking_speed(norm, 0., self.speed, limits.max_acceleration_mps2,
                              settings.max_jerk_mps3, math.hypot(*self.translation.acceleration),
                              2/self.translation.bandwidth))
        if reference.pure_rotation and norm <= limits.position_tolerance_m*.5:
            speed = 0.
        desired = tuple(v/norm*speed for v in error) if norm else (0.0, 0.0, 0.0)
        self.translation.step(desired, dt)
        yaw_speed = min(limits.yaw_gain*abs(yaw_error), braking_speed(abs(yaw_error), 0.,
            limits.max_yaw_rate_deg_s, limits.max_yaw_acceleration_deg_s2,
            settings.max_yaw_jerk_deg_s3, self.rotation.acceleration[0], 2/self.rotation.bandwidth))
        self.rotation.step((math.copysign(yaw_speed, yaw_error),), dt)
        return self.velocity, self.yaw_rate

