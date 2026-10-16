"""Rejected/missing detector evidence cannot redirect an already bound stage."""
from dataclasses import replace
import pytest

from planner.errors import ProtocolError
from tests.fakes.visual_goal import wall_observation, INVISIBLE
from tests.integration.test_visual_goal_workflow import setup
from tests.integration.test_continuous_visual_goal import continuous_setup
from tests.integration.test_visual_goal_radius_pause import Trace


@pytest.mark.parametrize('continuous',[False,True])
@pytest.mark.parametrize('failure',['wrong_car','invisible','protocol'])
def test_detection_failure_keeps_flying_to_original_memory_and_can_complete(continuous,failure):
    if continuous:
        w,stage,vehicle,planner,trace,clock=continuous_setup(target_x=30)
        source=w.source;observer=w.continuous.targets.observer
    else:
        w,stage,source,observer,planner,vehicle,clock=setup(target_x=30)
        trace=Trace();w.trace_factory=lambda:trace
    capture=source.capture
    def capture_with_wrong_depth(**kwargs):
        obs=capture(**kwargs)
        if failure=='wrong_car' and getattr(source,'counter',getattr(source,'captures',0))>1:
            obs=wall_observation(60-obs.vehicle_pose.x,obs.vehicle_pose,clock=clock,stamp=obs.timestamp_ns)
        return obs
    source.capture=capture_with_wrong_depth
    original=observer.observe
    def observe(*args,**kwargs):
        goal=original(*args,**kwargs)
        if len(observer.calls)>1:
            if failure=='invisible':return INVISIBLE
            if failure=='protocol':raise ProtocolError('synthetic detection failure')
        return goal
    observer.observe=observe
    try:
        result=w.run(stage)
        assert result.success,result.error
        assert len(observer.calls)>1 and len(planner.calls)>=2
        for obs,instruction,context in planner.calls:
            assert context['task_context'].geometry.world_point[0]==pytest.approx(30)
            assert context['task_context'].target_capture_id==planner.calls[0][2]['task_context'].target_capture_id
        checker=w.verification.checker
        assert checker.calls[-1].target_memory.world_point[0]==pytest.approx(30)
        assert checker.calls[-1].goal.bbox is None
        assert result.value.final_distance_m<=4.7
        assert not result.value.remaining_motion.segments
        name='target_update_failed' if failure=='protocol' else 'target_update_rejected'
        assert any(n==name for n,d in trace.events)
        assert not any(n=='pause_requested' and d['reason']=='target_identity_uncertain' for n,d in trace.events)
    finally:w.close()


def test_invalid_moving_camera_geometry_is_not_treated_as_optional_target_loss():
    w,stage,vehicle,planner,trace,clock=continuous_setup(target_x=30)
    source=w.source
    capture=source.capture_moving
    source.capture_moving=lambda **kwargs:replace(capture(**kwargs),depth_kind='unknown')
    try:
        result=w.run(stage)
        assert not result.success and result.value.status.value=='observation_error'
        assert not result.value.completion_confirmed
        assert vehicle.hovers>=1
    finally:w.close()
