"""Real local geometry with fake detector/vehicle: 4m completion and vertical motion."""
from tests.fakes.target_detection import detection_json
from dataclasses import replace
from unittest.mock import Mock

import pytest

from planner.adapters.target_detection.client import parse_target_detection
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.domain.pose import RelativePoseDelta
from planner.domain.visual_goal_completion import VisualGoalTriggerRules
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.services.motion import SynchronizedMotionPlanner
from planner.domain.motion import MotionLimits
from planner.domain.trajectory import WorldTrajectory
from tests.integration.test_visual_goal_radius_pause import StepVehicle, Trace
from tests.integration.test_visual_goal_window import enable_window
from tests.integration.test_visual_goal_workflow import setup
from tests.fakes.visual_goal import CompletionChecker


def set_radius(workflow,radius):
    workflow.trigger_rules=VisualGoalTriggerRules(radius_m=radius)
    workflow.flight.trigger_rules=workflow.trigger_rules


@pytest.mark.parametrize("distance",[3.0,3.5])
@pytest.mark.parametrize("sliding",[False,True])
def test_safe_interior_completes_without_flying_toward_four_meter_boundary(distance,sliding):
    workflow,stage,source,_,planner,vehicle,_=setup(target_x=distance)
    set_radius(workflow,4)
    if sliding:
        enable_window(workflow)
    try:
        result=workflow.run(stage)
        assert result.success,result.error
        assert not planner.calls and not vehicle.segments
        assert result.value.final_distance_m==pytest.approx(distance,abs=.01)
        assert result.value.completion_confirmed and source.captures==2
        assert not result.value.remaining_motion.segments
    finally:
        workflow.close()


@pytest.mark.parametrize("sliding",[False,True])
@pytest.mark.parametrize("first_completed",[False,True])
def test_four_meter_radius_pauses_then_confirms_after_safe_vertical_approach(sliding,first_completed):
    limits=replace(VisualGoalLimits())
    workflow,stage,source,_,planner,_,clock=setup(limits=limits)
    set_radius(workflow,4)
    vehicle=StepVehicle()
    source.vehicle=vehicle
    workflow.execution.vehicle=vehicle
    detector=Mock()
    detector.detect.return_value=parse_target_detection(detection_json(((0.1, 0.1), (0.9, 0.9))))
    workflow.goal_observer=BBoxGoalObserver(detector, 20, clock)
    planner.points=(RelativePoseDelta(1.5,0,-1.2,0),RelativePoseDelta(1.5,0,0,0),
                    RelativePoseDelta(0,0,0,0),RelativePoseDelta(0,0,0,0),RelativePoseDelta(0,0,0,0))
    trace=Trace()
    workflow.trace_factory=lambda:trace
    workflow.verification.checker=CompletionChecker((True,) if first_completed else (False,True))
    if sliding:
        enable_window(workflow)
    try:
        result=workflow.run(stage)
        assert result.success,result.error
        assert abs(vehicle.pose.z+4)>1  # The removed default +/-1m gate would reject this motion.
        assert result.value.final_distance_m+.3<=4
        assert any(name=="radius_pause" for name,_ in trace.events)
        assert any(name=="completion_confirmed" for name,_ in trace.events)
        checks=[data for name,data in trace.events if name=="completion_check"]
        assert [data["completed"] for data in checks]==([True] if first_completed else [False,True])
        if not first_completed:
            assert checks[0]["retained_points"]>0
        assert not result.value.remaining_motion.segments
        if sliding:
            assert not workflow.window.queue.items and workflow.window.background.candidate is None
    finally:
        workflow.close()


