"""Whole-queue speed preview with yaw timing and jerk-aware braking distance."""
import math

from .continuous_reference import segment_length, xyz, is_rotation


def stopping_distance(speed, acceleration, jerk, current_acceleration=0., response_time=0.):
    # Conservatively allow a current acceleration to point along the motion.
    a = abs(current_acceleration)
    neutral_time = a/jerk
    neutral_distance = speed*neutral_time+a**3/(3*jerk*jerk)
    v = speed+a*a/(2*jerk)
    if v <= acceleration*acceleration/jerk:
        stop = v*math.sqrt(v/jerk)
    else:
        stop = .5*v*(v/acceleration+acceleration/jerk)
    return neutral_distance+stop+speed*response_time


def braking_speed(distance, end_speed, maximum_speed, acceleration, jerk,
                  current_acceleration=0., response_time=0.):
    if end_speed >= maximum_speed:
        return maximum_speed
    terminal = stopping_distance(end_speed, acceleration, jerk, response_time=response_time)
    low, high = 0., maximum_speed
    for _ in range(24):
        mid = (low+high)*.5
        required = stopping_distance(mid, acceleration, jerk, current_acceleration, response_time)-terminal
        if required <= max(0., distance):
            low = mid
        else:
            high = mid
    return low


def angular_duration(angle, limits, settings):
    # Bounds on derivatives of a unit quintic, for a conservative rest-to-rest
    # yaw timing budget. Adjacent segments may actually carry nonzero yaw rate.
    angle = abs(angle)
    if not angle:
        return 0.
    return max(1.875*angle/limits.max_yaw_rate_deg_s,
               math.sqrt(10/math.sqrt(3)*angle/limits.max_yaw_acceleration_deg_s2),
               (60*angle/settings.max_yaw_jerk_deg_s3)**(1/3))


def corner_speed(before, after, limits, settings, cruise, response_time):
    u = tuple(b-a for a, b in zip(xyz(before.start), xyz(before.end)))
    v = tuple(b-a for a, b in zip(xyz(after.start), xyz(after.end)))
    first, second = math.hypot(*u), math.hypot(*v)
    if min(first, second) <= 1e-9:
        return 0.
    cosine = max(-1., min(1., sum(a*b for a, b in zip(u, v))/(first*second)))
    half = math.acos(cosine)*.5
    if half < 1e-6:
        return cruise
    if math.cos(half) < 1e-6:
        return 0.
    radius = min(limits.position_tolerance_m/(1/math.cos(half)-1),
                 .45*min(first, second)/math.tan(half))
    # A velocity-direction change also needs time in the jerk-limited command
    # response. A geometric centripetal cap alone underestimates corner drift.
    transition = braking_speed(limits.position_tolerance_m/max(2*math.sin(half), 1e-9),
        0., cruise, limits.max_acceleration_mps2, settings.max_jerk_mps3,
        response_time=1.5*response_time)
    return min(cruise, math.sqrt(limits.max_acceleration_mps2*radius), transition)


def planned_speed(items, reference, limits, settings, cruise, smoother):
    items = tuple(item for index,item in enumerate(items)
                  if index == 0 or segment_length(item) > 1e-9 or is_rotation(item))
    speed, distance = cruise, 0.
    response = 2/smoother.bandwidth
    acceleration = math.hypot(*smoother.acceleration)
    for index, item in enumerate(items):
        length = segment_length(item)
        duration = angular_duration(item.yaw_delta_deg, limits, settings)
        segment_speed = min(cruise, length/duration) if duration else cruise
        if length <= 1e-9:
            segment_speed = 0.
        speed = min(speed, braking_speed(distance, segment_speed, cruise,
            limits.max_acceleration_mps2, settings.max_jerk_mps3, acceleration, response))
        distance += reference.remaining_head_m if index == 0 else length
        boundary = corner_speed(item, items[index+1], limits, settings, cruise, response) if index+1 < len(items) else 0.
        # Leave room to enter the final position tolerance without crossing the
        # tail at cruise speed. This scans all authorized points, not the carrot.
        available = max(0., distance-limits.position_tolerance_m*.5) if index+1 == len(items) else distance
        if 0 < boundary < cruise:
            # Reach the cornering speed before the geometric switch. Retain a
            # nonzero through speed; true stop barriers still brake to the node.
            preview = min(settings.max_lookahead_m, max(settings.min_lookahead_m,
                          cruise*settings.lookahead_time_s))
            available = max(0., available-preview)
        speed = min(speed, braking_speed(available, boundary, cruise,
            limits.max_acceleration_mps2, settings.max_jerk_mps3, acceleration, response))
    return speed
