"""Reusable arrival assessment without scene audit or detector dependency."""
import pytest
from planner.capabilities.approach import TargetApproach
from planner.domain.approach import ApproachSettings, ApproachStatus, ApproachTarget
from planner.domain.pose import WorldPose


@pytest.mark.parametrize("distance,inside", [(0, True), (.1, True), (2, True), (3.5, True), (3.8, False), (4.1, False)])
def test_arrival_has_an_upper_bound_and_no_body_clearance_lower_bound(distance, inside):
    approach = TargetApproach()
    result = approach.assess(WorldPose(0, 0, -4, 0), ApproachTarget((distance, 0, -4), True), ApproachSettings(4, .3))
    assert (result.status is ApproachStatus.WITHIN_RADIUS) is inside
    assert result.distance_m == distance


@pytest.mark.parametrize("distance", [None, True, -1, float("nan"), float("inf"), 10**500])
def test_invalid_distance_is_not_an_arrival(distance):
    assert not TargetApproach.distance_within_radius(distance, ApproachSettings(4, .3))


def test_unavailable_target_is_unconfirmed():
    result = TargetApproach().assess(WorldPose(0, 0, -4, 0), ApproachTarget(None, False), ApproachSettings(4, .3))
    assert result.status is ApproachStatus.UNCONFIRMED and result.distance_m is None


@pytest.mark.parametrize("point", [[1,2,3], (1,2), (True,0,0), (float("nan"),0,0), (10**500,0,0)])
def test_mutable_or_invalid_target_coordinates_are_rejected(point):
    with pytest.raises(ValueError):
        ApproachTarget(point, True)
