"""Anchor freshness: what makes an observation unusable, and how it is named."""

from types import SimpleNamespace

import pytest

from planner.domain import WorldPose
from planner.domain import VisualGoalStatus
from planner.domain import VisualGoalLimits
from planner.workflows.visual_goal_navigation.termination import (
    VisualGoalExitPolicy,
    VisualGoalFailure,
)


def observation(*, x=1.0, y=2.0, z=-3.0, yaw_deg=10.0, captured=100.0):
    return SimpleNamespace(
        pose=WorldPose(x, y, z, yaw_deg), captured_monotonic=captured
    )


def test_a_fresh_observation_on_the_same_pose_is_valid():
    policy = VisualGoalExitPolicy(VisualGoalLimits())
    observation_pose = observation()
    actual = WorldPose(1.0, 2.0, -3.0, 10.0)

    assert policy.anchor_violation(observation_pose, actual, 100.5) is None
    assert policy.anchor_valid(observation_pose, actual, 100.5) is True


def test_translation_past_the_limit_is_named_with_the_measured_distance():
    policy = VisualGoalExitPolicy(VisualGoalLimits())
    reason = policy.anchor_violation(
        observation(), WorldPose(1.8, 2.0, -3.0, 10.0), 101.0
    )

    assert reason == "moved 0.80m over 0.30m"


def test_yaw_past_the_limit_is_named_with_the_measured_turn():
    policy = VisualGoalExitPolicy(VisualGoalLimits())
    reason = policy.anchor_violation(
        observation(), WorldPose(1.0, 2.0, -3.0, 15.0), 101.0
    )

    assert reason == "turned 5.0deg over 2.0deg"


def test_an_observation_older_than_the_age_limit_is_named_as_stale():
    policy = VisualGoalExitPolicy(VisualGoalLimits(maximum_observation_age_s=5.0))
    reason = policy.anchor_violation(
        observation(captured=100.0), WorldPose(1.0, 2.0, -3.0, 10.0), 120.0
    )

    assert reason == "age 20.0s over 5s"


def test_an_observation_without_a_capture_time_is_never_valid():
    policy = VisualGoalExitPolicy(VisualGoalLimits())
    reason = policy.anchor_violation(
        observation(captured=0.0), WorldPose(1.0, 2.0, -3.0, 10.0), 100.0
    )

    assert reason == "observation has no capture time"


def test_repeated_drift_fails_the_stage_instead_of_refreshing_forever():
    policy = VisualGoalExitPolicy(VisualGoalLimits(maximum_anchor_refresh_failures=2))
    actual = WorldPose(9.0, 2.0, -3.0, 10.0)

    assert policy.anchor_valid(observation(), actual, 101.0) is False
    with pytest.raises(VisualGoalFailure) as failure:
        policy.anchor_valid(observation(), actual, 101.1)

    assert failure.value.status is VisualGoalStatus.PERCEPTION_UNRELIABLE