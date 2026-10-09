from dataclasses import replace
import math

import pytest

from config.basic_motion import basic_motion_config
from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.capabilities.basic_motion_safety import BasicMotionSafety
from planner.capabilities.completion.basic_motion import BasicMotionCompletion
from planner.domain.basic_motion import BasicMotionParameters, ConfirmedFreeSpace
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.domain.trajectory import MotionSegment, MotionTrajectory
from tests.fakes.visual_goal import wall_observation


@pytest.mark.parametrize("fields,expected", [
    ({"up_m": 12}, (0, 0, -12, 0)), ({"up_m": -3}, (0, 0, 3, 0)),
    ({"yaw_right_deg": -90}, (0, 0, 0, -90)), ({"yaw_right_deg": 180}, (0, 0, 0, 180)),
    ({"yaw_right_deg": -180}, (0, 0, 0, -180)), ({"right_m": 5}, (0, 5, 0, 0)),
    ({"right_m": -5}, (0, -5, 0, 0)), ({"forward_m": -2}, (-2, 0, 0, 0)),
    ({"forward_m": 8, "right_m": 5, "yaw_right_deg": 15}, (8, 5, 0, 15)),
])
def test_semantic_conversion(fields, expected):
    point = BasicMotionConverter().create_trajectory(BasicMotionParameters("explicit command", **fields)).points[0]
    assert (point.dx, point.dy, point.dz, point.dyaw_deg) == expected


@pytest.mark.parametrize("value", [True, "12", None, math.inf, math.nan])
def test_basic_parameters_reject_invalid_numeric_values(value):
    with pytest.raises(ValueError):
        BasicMotionParameters("up", up_m=value)


def test_no_empty_action_and_no_large_step_or_rotation():
    with pytest.raises(ValueError):
        BasicMotionParameters("vague motion")
    for parameters in (BasicMotionParameters("diagonal", forward_m=25, right_m=25),
                       BasicMotionParameters("multi-turn", yaw_right_deg=360)):
        with pytest.raises(ValueError):
            parameters.validate_limits(30, 180)


@pytest.mark.parametrize("raw", [
    {"free_space": {"confirmed": "true"}},
    {"free_space": {"confirmed": True}},
    {"free_space": {"clearance_radius_m": True}},
    {"free_space": {"minimum_world_ned_m": [0, 0, 0]}},
    {"free_space": {"minimum_world_ned_m": [0, 0, 0], "maximum_world_ned_m": [1, 1, 1]}},
    {"free_space": {"minimum_world_ned_m": [0, 0, 0], "maximum_world_ned_m": [math.inf, 5, 5]}},
    {"unknown": 1},
])
def test_safe_space_config_rejects_invalid_values(raw):
    with pytest.raises(ValueError):
        basic_motion_config(raw)


def test_box_check_includes_whole_body_and_pure_yaw():
    space = ConfirmedFreeSpace(True, (-10, -10, -20), (10, 10, 5), 1)
    checker = BasicMotionSafety(space)
    start = WorldPose(0, 0, -4, 0)
    for end in (WorldPose(8, 5, -16, 15), WorldPose(0, 0, -4, -180)):
        assert checker.check(MotionTrajectory((MotionSegment(start, end, "linear"),))).allowed
    for end in (WorldPose(9.1, 0, -4, 0), WorldPose(0, 0, -19.1, 0)):
        assert not checker.check(MotionTrajectory((MotionSegment(start, end, "linear"),))).allowed
    assert not BasicMotionSafety(ConfirmedFreeSpace()).check(
        MotionTrajectory((MotionSegment(start, start, "linear"),))
    ).allowed
    assert not checker.check(MotionTrajectory(())).allowed
    # Fixed world bounds do not follow a new stage's start.
    outside = WorldPose(30, 0, -4, 0)
    assert not checker.check(MotionTrajectory((MotionSegment(outside, outside, "linear"),))).allowed


@pytest.mark.parametrize("stopped,collision,pose,completed", [
    (True, False, WorldPose(2, 0, -4, 90), True),
    (False, False, WorldPose(2, 0, -4, 90), False),
    (True, True, WorldPose(2, 0, -4, 90), False),
    (True, False, WorldPose(3, 0, -4, 90), False),
    (True, False, WorldPose(2, 0, -4, 80), False),
])
def test_completion_requires_measured_pose_stop_and_no_collision(stopped, collision, pose, completed):
    observation = replace(wall_observation(pose=pose), stopped=stopped)
    decision = BasicMotionCompletion(MotionLimits()).evaluate(WorldPose(2, 0, -4, 90), observation, collision)
    assert decision.completed is completed
