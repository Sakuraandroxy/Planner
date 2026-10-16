"""No simulator or model server: physical partial progress and boolean decisions."""
from dataclasses import replace
import math
from pathlib import Path
import threading

import pytest

from planner.domain.pose import WorldPose, resolve_yaw_delta_deg, wrap_yaw_deg
from planner.domain.visual_goal import VisualGoalStatus
from planner.errors import MotionPaused, ProtocolError
from tests.fakes.components import FakeVehicle
from tests.fakes.visual_goal import CompletionChecker
from tests.integration.test_visual_goal_workflow import setup


class StepVehicle(FakeVehicle):
    def execute_segment(self, segment, *, budget=None, pose_checkpoint=None):
        self.segments.append(segment)
        start, end = self.pose, segment.end
        yaw = resolve_yaw_delta_deg(start.yaw_deg, end.yaw_deg, segment.yaw_delta_deg)
        for index in range(41):
            fraction = index/40
            measured_yaw = start.yaw_deg+yaw*fraction
            self.pose = WorldPose(start.x+(end.x-start.x)*fraction, start.y+(end.y-start.y)*fraction,
                                  start.z+(end.z-start.z)*fraction, wrap_yaw_deg(measured_yaw))
            if pose_checkpoint is not None and pose_checkpoint(self.pose, measured_yaw):
                raise MotionPaused(self.pose, measured_yaw)


class Trace:
    def __init__(self):
        self.path = Path("test-only-trace")
        self.events = []

    def event(self, name, **data):
        self.events.append((name, data))

    def observation(self, *args):
        pass

    def target_geometry(self, *args):
        pass


def partial_setup(answers=(True,), clearance_radius=6):
    from planner.domain.visual_goal_limits import VisualGoalLimits
    # Stepwise control permits an actual mid-segment radius pause.
    workflow, stage, source, observer, planner, vehicle, clock = setup(
        limits=replace(VisualGoalLimits()))
    partial = StepVehicle()
    source.vehicle = partial
    workflow.execution.vehicle = partial
    checker = CompletionChecker(answers)
    workflow.verification.checker = checker
    trace = Trace()
    workflow.trace_factory = lambda: trace
    return workflow, stage, source, planner, partial, checker, trace


def test_true_clears_retained_points_only_after_stopped_fresh_completion():
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup()
    result = workflow.run(stage)
    assert result.success, result.error
    pauses = [data for name, data in trace.events if name == "radius_pause"]
    assert pauses and 1 <= pauses[0]["pose"].x <= 1.1  # First control sample inside the estimated radius.
    assert pauses[0]["remaining_points"] == 5  # Current segment tail plus four NEW points remain.
    confirmed = [data for name, data in trace.events if name == "completion_confirmed"]
    assert confirmed[0]["remaining_points"] > 0
    assert result.value.completion_confirmed and not result.value.remaining_motion.segments
    assert checker.calls[0].observation.stopped and checker.calls[0].confirmation_count >= 2
    assert checker.calls[0].geometry.distance_m <= 4.7
    assert source.captures > 2 and len(planner.calls) == 1


def test_serial_starts_planning_between_radii_then_checks_after_stop_drift():
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup()
    workflow.trigger_rules = replace(workflow.trigger_rules, radius_m=7, pause_radius_m=6.5)
    workflow.flight.trigger_rules = workflow.trigger_rules
    # Target starts at 6m: move the start back so initial distance is 6.8m.
    vehicle.pose = replace(vehicle.pose, x=-.8)
    capture = source.capture
    drifted = False
    def capture_after_stop(**kwargs):
        nonlocal drifted
        if vehicle.segments and not drifted:
            vehicle.pose = replace(vehicle.pose, x=vehicle.pose.x-.15)
            drifted = True
        return capture(**kwargs)
    source.capture = capture_after_stop
    result = workflow.run(stage)
    assert result.success, result.error
    assert len(planner.calls) == 1  # 6.8m alone must not start completion.
    assert 6.5 < checker.calls[0].geometry.distance_m <= 7
    assert not result.value.remaining_motion.segments


def test_false_resumes_original_world_goal_and_never_reexecutes_full_relative_delta():
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup((False, True))
    owner = threading.get_ident()
    original = vehicle.execute_segment
    def execute(*args, **kwargs):
        assert threading.get_ident() == owner
        return original(*args, **kwargs)
    vehicle.execute_segment = execute
    result = workflow.run(stage)
    assert result.success, result.error
    checks = [data for name, data in trace.events if name == "completion_check"]
    assert [check["completed"] for check in checks] == [False, True]
    assert checks[0]["retained_points"] > 0
    assert len(planner.calls) == 1  # The remaining batch was retained through false.
    assert all(segment.end.x == 2 for segment in vehicle.segments)
    assert 0 < vehicle.segments[1].start.x < 2
    assert vehicle.pose.x <= 2.01
    assert len(vehicle.segments) <= 4  # Radius latch prevents a zero-motion pause loop.


def test_invalid_completion_preserves_remaining_points_and_fails_stopped():
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup()
    def invalid(*args, **kwargs):
        raise ProtocolError("malformed completion")
    checker.check = invalid
    result = workflow.run(stage)
    assert not result.success and result.value.status is VisualGoalStatus.MODEL_ERROR
    assert result.value.remaining_motion.segments and not result.value.completion_confirmed
    assert vehicle.cancelled and vehicle.hovered
    assert not any(name == "completion_confirmed" for name, _ in trace.events)


def test_completion_uses_current_images_and_memory_without_another_detection():
    from tests.fakes.visual_goal import INVISIBLE, goal_observation
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup()
    observe = workflow.goal_observer.observe
    def lost(*args, **kwargs):
        return goal_observation(INVISIBLE) if source.captures > 1 else observe(*args, **kwargs)
    workflow.goal_observer.observe = lost
    result = workflow.run(stage)
    assert result.success
    assert not result.value.remaining_motion.segments
    assert len(vehicle.segments) >= 1 and len(checker.calls) == 1
    assert checker.calls[0].target_memory is not None
    assert checker.calls[0].goal.bbox is None




def test_drift_during_true_reply_discards_boolean_and_keeps_remaining_flight():
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup()
    def drifting(*args, **kwargs):
        vehicle.pose = replace(vehicle.pose, y=vehicle.pose.y+0.5)
        return True
    checker.check = drifting
    result = workflow.run(stage)
    assert not result.success and not result.value.completion_confirmed
    assert result.value.remaining_motion.segments
    assert any(name == "completion_discarded" for name, _ in trace.events)


def test_timed_out_completion_is_quarantined_and_cannot_start_another_stage():
    import time
    from planner.services.model_calls import ModelCallService
    from planner.domain.visual_goal_limits import VisualGoalLimits
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup()
    workflow.flight.limits = replace(workflow.limits, model_request_timeout_s=0.02)
    workflow.verification.calls = ModelCallService(time.monotonic)
    release, finished = threading.Event(), threading.Event()
    def blocked(*args, **kwargs):
        try:
            release.wait(2)
            return True
        finally:
            finished.set()
    checker.check = blocked
    try:
        result = workflow.run(stage)
        assert result.value.status is VisualGoalStatus.TIMEOUT
        assert result.value.remaining_motion.segments and not result.value.completion_confirmed
        assert workflow.verification.calls.has_pending_call
        captures = source.captures
        next_result = workflow.run(replace(stage, stage_id="next_stage"))
        assert not next_result.success and source.captures == captures
        assert not any(name == "completion_confirmed" for name, _ in trace.events)
    finally:
        release.set()
        assert finished.wait(1)
