"""Re-anchor retained world goals without repeating already executed deltas."""
from planner.domain.pose import wrap_yaw_deg
from planner.domain.trajectory import WorldTrajectory


def retained_world_trajectory(pending, actual):
    if not pending.segments:
        raise ValueError("no retained trajectory")
    first = pending.segments[0]
    # Remaining signed yaw already excludes the rotation measured before pause.
    remaining_yaw = first.yaw_delta_deg
    if remaining_yaw is None:
        remaining_yaw = wrap_yaw_deg(first.end.yaw_deg-first.start.yaw_deg)
    deltas = (remaining_yaw+wrap_yaw_deg(first.start.yaw_deg-actual.yaw_deg), *(
        segment.yaw_delta_deg if segment.yaw_delta_deg is not None else
        wrap_yaw_deg(segment.end.yaw_deg-segment.start.yaw_deg) for segment in pending.segments[1:]))
    return WorldTrajectory(actual, tuple(segment.end for segment in pending.segments), tuple(deltas))
