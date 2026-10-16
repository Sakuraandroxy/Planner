"""Direct model route execution and current target-center updates, offline only."""
from dataclasses import replace
from unittest.mock import Mock

import pytest

from planner.adapters.target_detection.client import parse_target_detection
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.domain.pose import RelativePoseDelta
from planner.domain.visual_goal import VisualGoalStatus
from planner.domain.visual_goal_limits import VisualGoalLimits
from tests.fakes.components import FakeVehicle
from tests.fakes.visual_goal import CompletionChecker
from tests.fakes.target_detection import detection_json
from tests.integration.test_visual_goal_workflow import setup
from tests.integration.test_visual_goal_window import enable_window
from tests.integration.test_visual_goal_radius_pause import Trace


@pytest.mark.parametrize("sliding",[False,True])
def test_all_five_points_execute_beyond_old_distance_time_and_view_limits(sliding):
    workflow,stage,source,_,planner,_,clock=setup(target_x=7.5,
        limits=replace(VisualGoalLimits(),maximum_planning_rounds=1))
    class TimedVehicle(FakeVehicle):
        def execute_segment(self,segment,**kwargs):
            clock.sleep(segment.duration_s)
            super().execute_segment(segment,**kwargs)
    vehicle=TimedVehicle()
    source.vehicle=workflow.execution.vehicle=vehicle
    planner.points=(RelativePoseDelta(.6,3,0,0),)*5
    trace=Trace()
    workflow.trace_factory=lambda:trace
    if sliding: enable_window(workflow)
    try:
        result=workflow.run(stage)
        assert result.value.status is VisualGoalStatus.TIMEOUT  # It executes the route, which misses the locked target.
        assert len(vehicle.segments)==5
        assert vehicle.pose.x==pytest.approx(3) and vehicle.pose.y==pytest.approx(15)
        assert sum(segment.duration_s for segment in vehicle.segments)>8
        assert result.value.path_length_m>6
        assert not result.value.completion_confirmed and not result.value.remaining_motion.segments
        plans=[data for name,data in trace.events if name=="plan"]
        assert len(plans)==1 and len(plans[0]["motion"].segments)==5
        assert plans[0]["execution_policy"]=="direct_model_trajectory"
        assert "selection" not in plans[0]
    finally:
        workflow.close()


@pytest.mark.parametrize("sliding",[False,True])
def test_large_heading_changes_do_not_discard_later_valid_points(sliding):
    limits=replace(VisualGoalLimits(),maximum_planning_rounds=1)
    workflow,stage,_,_,planner,vehicle,_=setup(target_x=20,limits=limits)
    planner.points=(RelativePoseDelta(0,0,0,45),)*5
    if sliding: enable_window(workflow)
    try:
        result=workflow.run(stage)
        assert not result.success  # Planning limit ends the deliberately non-approaching task.
        assert len(vehicle.segments)==5
        assert all(segment.yaw_delta_deg==45 for segment in vehicle.segments)
        assert result.value.status is VisualGoalStatus.TIMEOUT
    finally:
        workflow.close()


def test_changed_depth_after_binding_does_not_replace_memory_for_completion():
    workflow,stage,source,_,planner,vehicle,clock=setup(target_x=6)
    detector=Mock()
    detector.detect.return_value=parse_target_detection(detection_json())
    workflow.goal_observer=BBoxGoalObserver(detector,20,clock)
    # Later center depth changes by 2m; memory stays at the first static point.
    capture=source.capture
    def current_capture(**kwargs):
        if source.captures>=1: source.target_x=4
        return capture(**kwargs)
    source.capture=current_capture
    workflow.verification.checker=CompletionChecker()
    try:
        result=workflow.run(stage)
        assert result.success,result.error
        requests=[call.args[0] for call in detector.detect.call_args_list]
        assert requests[0].reference_rgb is None
        assert all(request.reference_rgb is not None for request in requests[1:])
        completion=workflow.verification.checker.calls[-1]
        assert completion.geometry.world_point[0]==pytest.approx(6,abs=.01)
        assert completion.target_memory.anchor_world_point[0]==pytest.approx(6,abs=.01)
        assert result.value.final_distance_m==pytest.approx(4,abs=.01)
    finally:
        workflow.close()
