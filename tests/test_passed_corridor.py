"""'Passed endpoint' corridor: a widened lateral band for non-holonomic vehicles.

Regression for the overshoot -> reverse -> stuck failure: a rover that drifts
~0.36 m sideways while crossing an endpoint must advance the queue, not reverse
back to a point it cannot strafe onto.
"""

import pytest

from planner.domain import WorldPose
from planner.domain import QueuedWaypoint
from planner.runtime import reached
from planner.domain import MotionLimits as ConfiguredMotionLimits
from planner.simulators.projectairsim.rover import (
    ROVER_PASSED_LATERAL_TOLERANCE_M,
    rover_tracking_limits,
)
from planner.simulators.projectairsim.motion import tracking_limits


def _overshot_waypoint() -> QueuedWaypoint:
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    end = WorldPose(4.0, 0.0, 0.0, 0.0)
    return QueuedWaypoint(1, start, end, 0.0)


#: Crossed the endpoint (progress 4.14 m > 4.0 m) and drifted 0.355 m sideways;
#: 0.382 m from the endpoint, so neither ball nor the 0.3 m corridor catches it.
_OVERSHOT_POSE = WorldPose(4.14, 0.355, 0.0, 0.0)


def test_overshoot_inside_the_narrow_corridor_is_not_reached():
    waypoint = _overshot_waypoint()
    assert reached(waypoint, _OVERSHOT_POSE, 0.0, 0.0, 0.3, 2.0) is False


def test_rover_width_corridor_consumes_the_same_overshoot():
    waypoint = _overshot_waypoint()
    assert reached(waypoint, _OVERSHOT_POSE, 0.0, 0.0, 0.3, 2.0,
                   passed_lateral_tolerance=0.6) is True


def test_passed_corridor_defaults_to_the_reach_ball():
    waypoint = _overshot_waypoint()
    narrow = reached(waypoint, _OVERSHOT_POSE, 0.0, 0.0, 0.3, 2.0, passed_lateral_tolerance=None)
    assert narrow == reached(waypoint, _OVERSHOT_POSE, 0.0, 0.0, 0.3, 2.0)


def test_overshoot_still_requires_aligned_yaw():
    waypoint = _overshot_waypoint()
    # Wide corridor alone must not consume a still-turning waypoint.
    assert reached(waypoint, _OVERSHOT_POSE, 10.0, 0.0, 0.3, 2.0,
                   passed_lateral_tolerance=0.6) is False


def test_rover_tracking_limits_widen_the_corridor_drone_keeps_the_default():
    rover = rover_tracking_limits(None)
    assert rover.passed_lateral_tolerance_m == pytest.approx(ROVER_PASSED_LATERAL_TOLERANCE_M)
    # Config can only raise it, never narrow it below the physical rover width.
    narrowed = rover_tracking_limits(ConfiguredMotionLimits(passed_lateral_tolerance_m=0.2))
    assert narrowed.passed_lateral_tolerance_m == pytest.approx(ROVER_PASSED_LATERAL_TOLERANCE_M)
    # The holonomic path (drone) is untouched.
    assert tracking_limits(None).passed_lateral_tolerance_m == pytest.approx(0.3)
