from dataclasses import replace

import numpy as np
import pytest

from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.domain.trajectory import WorldTrajectory
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.services.motion import SynchronizedMotionPlanner
from planner.services.trajectory_safety import TrajectorySafetyChecker
from tests.fakes.visual_goal import wall_observation


def select(poses, *, limits=None, obstacle=False, unknown=False):
    obs = wall_observation(distance=6, pose=WorldPose(0, 0, -4, 0))
    limits = limits or VisualGoalLimits()
    checker = TrajectorySafetyChecker(DepthGeometry(0.5, 200, True), limits, MotionLimits(), True)
    free = checker.begin(obs)
    if obstacle:
        obs.depth_meters[64, 64] = 2.7
    if unknown:
        obs.depth_meters[:] = np.nan
    motion = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(WorldTrajectory(obs.vehicle_pose, tuple(poses)))
    return checker.select(motion, obs, obs.vehicle_pose, free, (6, 0, -4), 3.3)


def test_prefix_does_not_silently_clip_an_overshooting_endpoint():
    poses = [WorldPose(2, 0, -4, 0), WorldPose(4, 0, -4, 0)]
    chosen = select(poses)
    assert len(chosen.motion.segments) == 1
    assert chosen.motion.segments[0].end == poses[0]


def test_single_near_obstacle_in_corridor_is_not_hidden_by_median():
    chosen = select([WorldPose(2, 0, -4, 0)], obstacle=True)
    assert not chosen.motion.segments
    assert "obstacle" in chosen.reason


@pytest.mark.parametrize("pose", [WorldPose(0, 0, -6, 0), WorldPose(2, 0, -4, 45), WorldPose(-6, 0, -4, 0)])
def test_height_turn_and_blind_translation_are_rejected(pose):
    assert not select([pose]).motion.segments


def test_pure_yaw_requires_reobservation_before_following_segment():
    chosen = select([WorldPose(0, 0, -4, 30), WorldPose(1, 0, -4, 30)])
    assert len(chosen.motion.segments) == 1
    assert "reobservation" in chosen.reason


def test_whole_segment_cannot_exceed_smooth_time_horizon():
    chosen = select([WorldPose(2, 0, -4, 0)], limits=replace(VisualGoalLimits(), maximum_round_motion_s=1))
    assert not chosen.motion.segments
    assert "horizon" in chosen.reason


def test_unknown_space_outside_initial_certified_bubble_is_not_free():
    chosen = select([WorldPose(2, 0, -4, 0)], unknown=True)
    assert not chosen.motion.segments


def test_anchor_displacement_and_tracking_error_have_separate_margins():
    limits = VisualGoalLimits()
    assert limits.clearance_radius_m == pytest.approx(2.1)
    with pytest.raises(ValueError, match="safe standoff"):
        limits.validate_standoff(3, 200)  # 2m lower band is inside the 2.1m envelope.


def test_additional_surface_constrains_safe_prefix_independently_of_anchor():
    obs = wall_observation(distance=6)
    checker = TrajectorySafetyChecker(DepthGeometry(.5, 200, True), VisualGoalLimits(), MotionLimits(), True)
    motion = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(WorldTrajectory(
        obs.vehicle_pose, (WorldPose(2, 0, -4, 0),)))
    free = checker.begin(obs)
    assert checker.select(motion, obs, obs.vehicle_pose, free, (6, 0, -4), 3.3).motion.segments
    chosen = checker.select(motion, obs, obs.vehicle_pose, free, (6, 0, -4), 3.3,
                            additional_standoff_points=((4, 1, -4),))
    assert not chosen.motion.segments and "standoff" in chosen.reason
