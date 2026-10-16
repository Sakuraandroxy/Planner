"""Direct depth retains pause/complete semantics with model-provided named points."""
from tests.fakes.target_detection import detection_json
from dataclasses import replace
from unittest.mock import Mock

import numpy as np
import pytest

from planner.adapters.target_detection.client import parse_target_detection
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.domain.visual_goal_limits import VisualGoalLimits
from tests.fakes.visual_goal import CompletionChecker
from tests.integration.test_visual_goal_radius_pause import StepVehicle, Trace
from tests.integration.test_visual_goal_window import enable_window
from tests.integration.test_visual_goal_workflow import setup


def workflow_with_direct_depth(sliding):
    limits = replace(VisualGoalLimits())
    workflow, stage, source, _, planner, _, clock = setup(limits=limits)
    vehicle = StepVehicle()
    source.vehicle = vehicle
    workflow.execution.vehicle = vehicle
    detector = Mock()
    detector.detect.return_value = parse_target_detection(detection_json(((0.25, 0.25), (0.75, 0.75))))
    workflow.goal_observer = BBoxGoalObserver(detector, 20, clock)
    checker = CompletionChecker((False, True))
    workflow.verification.checker = checker
    trace = Trace()
    workflow.trace_factory = lambda: trace
    if sliding:
        enable_window(workflow)
    return workflow, stage, source, planner, vehicle, checker, trace


@pytest.mark.parametrize("sliding", [False, True])
def test_direct_depth_keeps_pending_after_false_and_clears_only_on_true(sliding):
    workflow, stage, _, planner, vehicle, checker, trace = workflow_with_direct_depth(sliding)
    try:
        result = workflow.run(stage)
        assert result.success, result.error
        checks = [data for name, data in trace.events if name == "completion_check"]
        assert [item["completed"] for item in checks] == [False, True]
        assert checks[0]["retained_points"] > 0
        assert all(segment.end.x == 2 for segment in vehicle.segments)
        assert vehicle.pose.x <= 2.01 and len(checker.calls) == 2
        assert planner.calls and result.value.completion_confirmed
        assert not result.value.remaining_motion.segments
        if sliding:
            assert not workflow.window.queue.items and workflow.window.background.candidate is None
    finally:
        workflow.close()


@pytest.mark.parametrize("sliding", [False, True])
def test_false_completion_with_missing_current_target_depth_preserves_pending(sliding):
    workflow, stage, source, _, vehicle, checker, _ = workflow_with_direct_depth(sliding)
    rules = replace(workflow.trigger_rules, maximum_completion_checks=1)
    workflow.trigger_rules = workflow.flight.trigger_rules = rules
    capture = source.capture

    def invalid_after_first(*args, **kwargs):
        observation = capture(*args, **kwargs)
        if source.captures > 1:
            observation.depth_meters[64, 64] = np.nan
        return observation

    source.capture = invalid_after_first
    try:
        result = workflow.run(stage)
        assert not result.success and not result.value.completion_confirmed
        assert result.value.remaining_motion.segments
        assert vehicle.hovered and len(checker.calls) == 1
        assert checker.calls[0].target_memory is not None and checker.calls[0].goal.bbox is None
        if sliding:
            assert workflow.window.queue.items
    finally:
        workflow.close()
