"""Measured lane-frame prior used by road-following planning.

These values come from the simulator's HD map, not from the RGB/depth model.
They give the planner a lane centre, tangent and boundaries that metric depth
cannot observe, and are always reported in the current body frame so the
five-action relative protocol can consume them directly.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


def _finite(value: float) -> bool:
    """Reject bools, non-numeric values and non-finite numbers."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class LaneLookaheadPoint:
    """One lane-centre sample ahead of the ego in the current body frame.

    ``forward_m``/``right_m`` are the body-frame offsets of the lane centre,
    ``bearing_deg`` is the angle of that point from the current forward axis
    (positive right), and ``yaw_delta_deg`` is the lane tangent at that point
    relative to the current vehicle yaw (positive right). The latter is the
    cumulative turn target by that distance, matching the incremental protocol.
    """

    distance_m: float
    forward_m: float
    right_m: float
    bearing_deg: float
    yaw_delta_deg: float

    def __post_init__(self) -> None:
        if not all(
            _finite(value)
            for value in (
                self.distance_m,
                self.forward_m,
                self.right_m,
                self.bearing_deg,
                self.yaw_delta_deg,
            )
        ):
            raise ValueError("lane lookahead contains a non-finite value")
        if self.distance_m < 0.0:
            raise ValueError("lane lookahead distance must be non-negative")


@dataclass(frozen=True)
class LaneReference:
    """Vehicle pose relative to one measured driving lane.

    ``lateral_offset_m`` is positive to the right of the lane centre.
    ``heading_error_deg`` is the lane tangent minus the vehicle yaw, positive
    when the lane points right of the car. Boundary distances are along the
    lane's right axis; a negative value means the ego has crossed that
    boundary and is outside the lane.
    """

    road_id: int
    lane_id: int
    lane_width_m: float
    lateral_offset_m: float
    heading_error_deg: float
    distance_to_left_boundary_m: float
    distance_to_right_boundary_m: float
    junction_ahead: bool = False
    lookahead: tuple[LaneLookaheadPoint, ...] = ()

    def __post_init__(self) -> None:
        if type(self.road_id) is not int or type(self.lane_id) is not int:
            raise ValueError("lane reference ids must be integers")
        if not all(
            _finite(value)
            for value in (
                self.lane_width_m,
                self.lateral_offset_m,
                self.heading_error_deg,
                self.distance_to_left_boundary_m,
                self.distance_to_right_boundary_m,
            )
        ):
            raise ValueError("lane reference contains a non-finite value")
        if self.lane_width_m <= 0.0:
            raise ValueError("lane reference width must be positive")
        if type(self.junction_ahead) is not bool:
            raise ValueError("lane reference junction flag must be boolean")
        if not isinstance(self.lookahead, tuple) or not all(
            isinstance(point, LaneLookaheadPoint) for point in self.lookahead
        ):
            raise ValueError("lane reference lookahead must be lookahead points")