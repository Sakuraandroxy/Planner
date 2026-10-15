"""Polyline following with continuous velocity and independent signed yaw."""
import math



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


def limit_vector(vector, maximum):
    norm = math.hypot(*vector)
    return tuple(v*min(1.0, maximum/norm) for v in vector) if norm else tuple(vector)


class ContinuousFollower:
    def __init__(self, speed_mps, limits):
        if isinstance(speed_mps, bool) or not math.isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError("continuous speed must be finite and positive")
        self.speed, self.limits = speed_mps, limits
        self.velocity, self.yaw_rate = (0.0, 0.0, 0.0), 0.0

    def reset(self, velocity=(0.0, 0.0, 0.0)):
        self.velocity, self.yaw_rate = tuple(velocity), 0.0

    def sample(self, items, pose, yaw, goal_yaw, dt):
        head = items[0]
        b = head.end
        difference = (b.x-pose.x, b.y-pose.y, b.z-pose.z)
        distance = math.hypot(*difference)
        tolerance = self.limits.position_tolerance_m
        target = (b.x, b.y, b.z)
        # Look through an intermediate point instead of commanding zero speed
        # there. The bounded carrot stays within the configured corner tolerance.
        if len(items) > 1 and distance <= 2*tolerance:
            end = items[1].end
            direction = (end.x-b.x, end.y-b.y, end.z-b.z)
            norm = math.hypot(*direction)
            if norm:
                target = tuple(v+d/norm*tolerance*.5 for v, d in zip(target, direction))
        error = tuple(v-p for v, p in zip(target, (pose.x, pose.y, pose.z)))
        norm = math.hypot(*error)
        speed = self.speed
        if len(items) == 1:
            speed = min(speed, math.sqrt(2*self.limits.max_acceleration_mps2*distance))
        # Remain at a position-only endpoint until its explicit yaw is achieved.
        if distance <= tolerance and abs(goal_yaw-yaw) > self.limits.yaw_tolerance_deg:
            speed = min(speed, self.limits.position_gain*distance)
        desired = tuple(v/norm*speed for v in error) if norm else (0.0, 0.0, 0.0)
        change = limit_vector(tuple(v-p for v, p in zip(desired, self.velocity)),
                              self.limits.max_acceleration_mps2*dt)
        self.velocity = tuple(v+d for v, d in zip(self.velocity, change))
        desired_yaw = max(-self.limits.max_yaw_rate_deg_s,
            min(self.limits.max_yaw_rate_deg_s, self.limits.yaw_gain*(goal_yaw-yaw)))
        maximum_change = self.limits.max_yaw_acceleration_deg_s2*dt
        self.yaw_rate += max(-maximum_change, min(maximum_change, desired_yaw-self.yaw_rate))
        return self.velocity, self.yaw_rate

