import math
import pytest
from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.pose import WorldPose


@pytest.mark.parametrize("fields,expected", [
    ({"up_m": 10}, WorldPose(0,0,-14,0)),
    ({"up_m": -3}, WorldPose(0,0,-1,0)),
    ({"right_m": 5}, WorldPose(0,5,-4,0)),
    ({"forward_m": -2}, WorldPose(-2,0,-4,0)),
    ({"forward_m": 100}, WorldPose(100,0,-4,0)),
    ({"yaw_right_deg": 450}, WorldPose(0,0,-4,90)),
])
def test_numeric_actions_convert_without_distance_or_angle_caps(fields,expected):
    assert BasicMotionConverter().create_target(BasicMotionParameters("move",**fields),WorldPose(0,0,-4,0))==expected


def test_translation_uses_measured_stage_heading_before_new_turn():
    target=BasicMotionConverter().create_target(
        BasicMotionParameters("move and turn",forward_m=5,right_m=2,yaw_right_deg=90),WorldPose(10,20,-4,90))
    assert (target.x,target.y,target.z,target.yaw_deg)==pytest.approx((8,25,-4,-180))


@pytest.mark.parametrize("value", [True,"12",None,math.inf,math.nan])
def test_wire_values_must_still_be_finite_numbers(value):
    with pytest.raises(ValueError): BasicMotionParameters("up",up_m=value)
