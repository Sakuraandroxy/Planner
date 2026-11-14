import time
from pathlib import Path

import numpy as np
import pytest

from tests.support import ue_camera_matrix
from planner.capabilities.approach import TargetApproach
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.capabilities.planning import GeometricTargetPlanner
from planner.capabilities.targeting import DepthGeometry
from planner.domain.mission import MissionStage, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.sliding_window import SlidingWindowSettings
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import (
    GoalObservation,
    GoalVisibility,
    VisualGoalNavigationParameters,
    VisualGoalStatus,
)
from planner.domain.visual_goal_completion import VisualGoalTriggerRules
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import MotionPaused
from planner.services.execution import ExecutionService
from planner.services.motion import SynchronizedMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows.visual_goal_navigation import VisualGoalNavigationWorkflow

BOX = (0.4, 0.4, 0.6, 0.6)


class FakeVehicle:
    def __init__(self, pose: WorldPose = WorldPose(0.0, 0.0, 0.0, 0.0)):
        self.pose = pose
        self.segments = 0

    def current_pose(self):
        return self.pose

    def execute_segment(self, segment, *, budget=None, pose_checkpoint=None):
        self.segments += 1
        self.pose = segment.end
        if pose_checkpoint is not None and pose_checkpoint(
            self.pose, self.pose.yaw_deg
        ):
            raise MotionPaused(self.pose, self.pose.yaw_deg)

    def hover(self):
        pass

    def cancel(self):
        pass

    def collision_state(self):
        return False


class FakeSource:
    """Uniform optical-depth plane in front of the vehicle."""

    def __init__(self, vehicle: FakeVehicle, target_x: float, depth_scale=1.0):
        self.vehicle = vehicle
        self.target_x = target_x
        self.depth_scale = depth_scale
        self.captures = 0
        self.timestamp_ns = 0
        self.intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)

    def capture(self, *, budget=None):
        self.captures += 1
        self.timestamp_ns += 1_000_000
        pose = self.vehicle.pose
        depth = np.full(
            (480, 640),
            max(0.0, self.target_x - pose.x) * self.depth_scale,
            dtype=np.float32,
        )
        return Observation(
            frame=self.captures,
            timestamp_s=self.captures * 0.01,
            pose=pose,
            rgb=np.zeros((480, 640, 3), dtype=np.uint8),
            depth_m=depth,
            intrinsics=self.intrinsics,
            camera_matrix=ue_camera_matrix(pose.x, pose.y, -pose.z, pose.yaw_deg),
            timestamp_ns=self.timestamp_ns,
            captured_monotonic=time.monotonic(),
            capture_id=f"capture-{self.captures}",
            stopped=True,
        )


def _visible_goal():
    return GoalObservation(
        GoalVisibility.VISIBLE,
        BOX,
        (0.5, 0.5),
        "the target",
        "fake observer",
    )


class FixedObserver:
    def __init__(self, goal=None, *, error=None):
        self.goal = goal or _visible_goal()
        self.error = error
        self.calls = 0

    def observe(
        self,
        observation,
        target_description,
        reference=None,
        *,
        budget=None,
        initial_pose=None,
        instruction=None,
    ):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return self.goal


class FixedPlanner:
    def __init__(self, points):
        self.points = tuple(points)
        self.calls = 0

    def plan(self, observation, instruction, **kwargs):
        self.calls += 1
        return RelativeTrajectory(self.points)


class ScriptedChecker:
    def __init__(self, *verdicts):
        self.verdicts = list(verdicts)
        self.requests = []

    def check(self, request, *, timeout_s):
        self.requests.append(request)
        return self.verdicts.pop(0) if self.verdicts else True


class FakeTrace:
    def __init__(self):
        self.path = Path("/tmp/fake-visual-goal-trace")
        self.events = []

    def event(self, event, **data):
        self.events.append((event, data))

    def observation(self, observation, goal):
        pass

    def target_geometry(self, observation, goal, geometry):
        pass


def _parameters():
    return VisualGoalNavigationParameters(
        instruction="drive to the target",
        target_description="the target",
    )


def _stage():
    return MissionStage("stage-1", TaskKind.VISUAL_GOAL_NAVIGATION, _parameters())


def _limits(**overrides):
    values = dict(
        confirmation_interval_s=0.01,
        model_request_timeout_s=5.0,
        model_retry_count=0,
        maximum_task_duration_s=30.0,
        maximum_unreliable_observations=2,
    )
    values.update(overrides)
    return VisualGoalLimits(**values)


def _workflow(
    source,
    vehicle,
    observer=None,
    planner=None,
    *,
    checker=None,
    limits=None,
    trigger_rules=None,
    trace_factory=None,
):
    limits = limits or _limits()
    rules = trigger_rules or VisualGoalTriggerRules(
        radius_m=7.0, pause_radius_m=6.5
    )
    motion_limits = MotionLimits()
    return VisualGoalNavigationWorkflow(
        source,
        observer or FixedObserver(),
        planner or GeometricTargetPlanner(),
        TrajectoryValidator(5, 2.0, 45.0),
        SynchronizedMotionPlanner(2.0, motion_limits),
        ExecutionService(vehicle),
        DepthGeometry(0.5, True),
        VisualGoalCompletion(limits, TargetApproach()),
        limits,
        trace_factory=trace_factory,
        completion_checker=checker or ScriptedChecker(True),
        trigger_rules=rules,
        sliding_window=SlidingWindowSettings(),
        motion_limits=motion_limits,
    )


def test_reaches_target_after_pause_and_two_confirmations():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    workflow = _workflow(source, vehicle)
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.REACHED
    assert result.completion_confirmed is True
    assert result.planning_calls == 1
    assert result.completion_checks == 1
    assert result.confirmation_count == 2
    assert result.path_length_m == pytest.approx(3.84, abs=1e-2)
    assert result.final_distance_m == pytest.approx(6.16, abs=1e-2)


def test_missing_target_with_search_disabled_is_blocked():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    observer = FixedObserver(
        GoalObservation(
            GoalVisibility.NOT_VISIBLE, None, None, "the target", "missing"
        )
    )
    workflow = _workflow(
        source, vehicle, observer, limits=_limits(search_enabled=False)
    )
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.BLOCKED
    assert result.path_length_m == 0.0


def test_invalid_depth_is_perception_unreliable():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0, depth_scale=0.0)
    workflow = _workflow(source, vehicle)
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.PERCEPTION_UNRELIABLE


def test_observer_failure_is_reported_as_model_error():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    workflow = _workflow(
        source, vehicle, FixedObserver(error=RuntimeError("detector failed"))
    )
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.MODEL_ERROR
    assert "detector failed" in result.reason


def test_zero_motion_twice_is_stuck():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    planner = FixedPlanner(tuple(RelativePoseDelta() for _ in range(5)))
    workflow = _workflow(source, vehicle, planner=planner)
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.STUCK
    assert result.planning_calls == 2


def test_planning_budget_exhaustion_is_timeout():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=100.0)
    workflow = _workflow(
        source,
        vehicle,
        limits=_limits(maximum_planning_rounds=1),
    )
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.TIMEOUT
    assert result.planning_calls == 1


def test_hold_checkpoint_cancels_the_task():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    workflow = _workflow(source, vehicle)

    def stop():
        raise KeyboardInterrupt

    workflow.execution.bind_hold_checkpoint(stop)
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.CANCELLED
    assert vehicle.segments == 0


def test_failed_completion_check_requires_new_confirmations():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    checker = ScriptedChecker(False, True)
    workflow = _workflow(source, vehicle, checker=checker)
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.REACHED
    assert result.completion_checks == 2
    assert len(checker.requests) == 2


def test_completion_check_budget_exhaustion_is_timeout():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    checker = ScriptedChecker(False, False, False)
    workflow = _workflow(
        source,
        vehicle,
        checker=checker,
        trigger_rules=VisualGoalTriggerRules(
            radius_m=7.0, pause_radius_m=6.5, maximum_completion_checks=1
        ),
    )
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    assert result.status is VisualGoalStatus.TIMEOUT
    assert result.completion_checks == 1


def test_trace_receives_task_events():
    vehicle = FakeVehicle()
    source = FakeSource(vehicle, target_x=10.0)
    trace = FakeTrace()
    workflow = _workflow(
        source, vehicle, trace_factory=lambda: trace
    )
    try:
        result = workflow.run(_stage()).value
    finally:
        workflow.close()
    names = [name for name, _ in trace.events]
    assert "task_start" in names
    assert "target_updated" in names
    assert "completion_confirmed" in names
    assert "result" in names
    assert result.status is VisualGoalStatus.REACHED
    assert result.run_directory == str(trace.path)