import time

import numpy as np
import pytest

from tests.support import ue_camera_matrix
from planner.capabilities.approach import TargetApproach
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.capabilities.completion.visual_goal_request import (
    require_visual_completion_evidence,
)
from planner.capabilities.completion.visual_goal_retained_motion import (
    retained_world_trajectory,
)
from planner.capabilities.completion.visual_goal_trigger import VisualGoalRadiusTrigger
from planner.capabilities.completion.visual_goal_verification import VisualGoalVerification
from planner.domain.approach import ApproachSettings, ApproachStatus, ApproachTarget
from planner.domain.execution import ExecutionBudget
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.domain.trajectory import MotionSegment, MotionTrajectory
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    VisualGoalNavigationParameters,
)
from planner.domain.visual_goal_completion import (
    VisualGoalCompletionInput,
    VisualGoalTargetSnapshot,
    VisualGoalTriggerRules,
)
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ProtocolError


def _observation(pose=WorldPose(0.0, 0.0, 0.0, 0.0), *, stopped=True) -> Observation:
    width, height = 8, 6
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        depth_m=np.full((height, width), 5.0, dtype=np.float32),
        intrinsics=CameraIntrinsics.from_horizontal_fov(width, height, 90.0),
        camera_matrix=ue_camera_matrix(pose.x, pose.y, -pose.z, pose.yaw_deg),
        timestamp_ns=10,
        captured_monotonic=1.0,
        capture_id="capture-1",
        stopped=stopped,
    )


def _goal(status=GoalVisibility.VISIBLE, same_target=True) -> GoalObservation:
    return GoalObservation(status, (0.4, 0.4, 0.6, 0.6), None, "the red car", "visible",
                           same_target, "model_surface_points", surface_points=())


def _geometry(distance=5.0, world_point=(5.0, 0.0, 0.0)) -> GoalGeometry:
    return GoalGeometry(True, "sampled", distance, world_point)


def _parameters() -> VisualGoalNavigationParameters:
    return VisualGoalNavigationParameters("drive to the red car", "the red car")


def test_target_approach_applies_the_uncertainty_margin():
    settings = ApproachSettings(5.0, 0.5)
    assert TargetApproach.distance_within_radius(4.5, settings) is True
    assert TargetApproach.distance_within_radius(4.6, settings) is False
    assert TargetApproach.distance_within_radius(None, settings) is False
    assert TargetApproach.distance_within_radius(float("nan"), settings) is False


def test_target_approach_assess_reports_status_and_distance():
    settings = ApproachSettings(5.0, 0.0)
    approach = TargetApproach()
    near = approach.assess(WorldPose(2.0, 0.0, 0.0, 0.0),
                           ApproachTarget((5.0, 0.0, 0.0), True), settings)
    assert near.status is ApproachStatus.WITHIN_RADIUS
    assert near.distance_m == pytest.approx(3.0)
    far = approach.assess(WorldPose(-1.0, 0.0, 0.0, 0.0),
                          ApproachTarget((5.0, 0.0, 0.0), True), settings)
    assert far.status is ApproachStatus.APPROACHING
    unknown = approach.assess(WorldPose(0.0, 0.0, 0.0, 0.0),
                              ApproachTarget(None, False), settings)
    assert unknown.status is ApproachStatus.UNCONFIRMED


def test_visual_goal_completion_requires_stopped_reliable_evidence():
    completion = VisualGoalCompletion(VisualGoalLimits(distance_uncertainty_margin_m=0.0))
    kwargs = dict(parameters=_parameters(), goal=_goal(), geometry=_geometry(),
                  initial_pose=WorldPose(0.0, 0.0, 0.0, 0.0), radius_m=7.0)
    assert completion.candidate(observation=_observation(), **kwargs) is True
    assert completion.candidate(observation=_observation(stopped=False), **kwargs) is False
    assert completion.candidate(
        observation=_observation(),
        **{**kwargs, "geometry": GoalGeometry(False, "no depth")},
    ) is False
    assert completion.candidate(
        observation=_observation(),
        **{**kwargs, "goal": _goal(same_target=False)},
    ) is False


def test_visual_goal_completion_binds_membership_of_known_surfaces():
    from planner.domain.target_state import TargetPositionState
    state = TargetPositionState("stage-1", (5.0, 0.0, 0.0), (5.0, 0.0, 0.0),
                                "capture-0", 1.0, 5,
                                anchor_surface_points_world=((5.0, 0.0, 0.0),),
                                surface_points_world=((5.0, 0.0, 0.0),))
    completion = VisualGoalCompletion(VisualGoalLimits(distance_uncertainty_margin_m=0.0))
    kwargs = dict(parameters=_parameters(), observation=_observation(), goal=_goal(),
                  initial_pose=WorldPose(0.0, 0.0, 0.0, 0.0), radius_m=7.0, target_state=state)
    assert completion.candidate(geometry=_geometry(), **kwargs) is True
    assert completion.candidate(geometry=_geometry(9.0, (9.0, 0.0, 0.0)), **kwargs) is False


def _completion_request(**overrides) -> VisualGoalCompletionInput:
    values = dict(
        stage_id="stage-1",
        parameters=_parameters(),
        observation=_observation(),
        initial_pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        goal=_goal(),
        geometry=_geometry(),
        rules=VisualGoalTriggerRules(radius_m=7.0),
        uncertainty_margin_m=0.0,
        height_tolerance_m=None,
        confirmation_count=2,
        required_confirmation_count=2,
        reference_description="the red car",
    )
    values.update(overrides)
    return VisualGoalCompletionInput(**values)


def test_require_visual_completion_evidence_admits_and_rejects():
    require_visual_completion_evidence(_completion_request())
    with pytest.raises(ProtocolError):
        require_visual_completion_evidence(_completion_request(confirmation_count=1))
    with pytest.raises(ProtocolError):
        require_visual_completion_evidence(
            _completion_request(geometry=_geometry(9.0)))
    with pytest.raises(ProtocolError):
        require_visual_completion_evidence(
            _completion_request(goal=_goal(GoalVisibility.NOT_VISIBLE)))
    with pytest.raises(ProtocolError):
        require_visual_completion_evidence("not a request")


def test_retained_world_trajectory_reanchors_on_the_actual_pose():
    pending = MotionTrajectory((
        MotionSegment(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(1.0, 0.0, 0.0, 0.0), "s"),
        MotionSegment(WorldPose(1.0, 0.0, 0.0, 0.0), WorldPose(2.0, 0.0, 0.0, 0.0), "s"),
    ))
    actual = WorldPose(0.5, 0.0, 0.0, 0.0)
    trajectory = retained_world_trajectory(pending, actual)
    assert trajectory.start == actual
    assert trajectory.poses == (WorldPose(1.0, 0.0, 0.0, 0.0), WorldPose(2.0, 0.0, 0.0, 0.0))
    assert trajectory.yaw_deltas_deg == (0.0, 0.0)
    with pytest.raises(ValueError):
        retained_world_trajectory(MotionTrajectory(()), actual)


def test_visual_goal_radius_trigger_pauses_once_then_rearms():
    trigger = VisualGoalRadiusTrigger(VisualGoalTriggerRules(radius_m=4.0), 45.0, lambda: 2.0)
    trigger.update(VisualGoalTargetSnapshot("stage-1", (5.0, 0.0, 0.0), 1.0, stage_bound=True))
    with pytest.raises(ValueError):
        trigger.update(VisualGoalTargetSnapshot("stage-2", (5.0, 0.0, 0.0), 1.0))
    assert trigger.should_pause(WorldPose(2.0, 0.0, 0.0, 0.0)) is True
    assert trigger.should_pause(WorldPose(2.0, 0.0, 0.0, 0.0)) is False
    assert trigger.should_pause(WorldPose(4.0, 0.0, 0.0, 0.0)) is True
    assert trigger.should_pause(WorldPose(9.1, 0.0, 0.0, 0.0)) is False


def test_visual_goal_radius_trigger_expires_a_stale_unbound_target():
    trigger = VisualGoalRadiusTrigger(VisualGoalTriggerRules(radius_m=4.0), 0.5, lambda: 10.0)
    trigger.update(VisualGoalTargetSnapshot("stage-1", (5.0, 0.0, 0.0), 1.0))
    assert trigger.should_pause(WorldPose(0.0, 0.0, 0.0, 0.0)) is True
    assert trigger.last_reason == "target_reference_expired"


class _Checker:
    def __init__(self, values):
        self.values = list(values)
        self.requests = []

    def check(self, request, *, timeout_s):
        self.requests.append(request)
        value = self.values.pop(0)
        if isinstance(value, BaseException):
            raise value
        return value


def _verify(checker, retry_count=1):
    service = VisualGoalVerification(checker, time.monotonic)
    budget = ExecutionBudget(time.monotonic() + 5.0)
    try:
        return service.check(_completion_request(), budget, 1.0, retry_count,
                             lambda: None, time.monotonic)
    finally:
        service.calls.close()


def test_visual_goal_verification_returns_only_booleans_and_freezes_input():
    checker = _Checker([True])
    assert _verify(checker) is True
    assert checker.requests[0].observation.depth_m.flags.writeable is False
    with pytest.raises(ProtocolError):
        _verify(_Checker([1, 1]))
    with pytest.raises(ProtocolError):
        _verify(_Checker([ProtocolError("bad"), ProtocolError("bad")]))


def test_visual_goal_verification_retries_a_protocol_error():
    checker = _Checker([ProtocolError("transient"), True])
    assert _verify(checker, retry_count=1) is True
    assert len(checker.requests) == 2