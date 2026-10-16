"""A static target anchor cannot walk to another object through small updates."""
from dataclasses import replace
import pytest

from planner.capabilities.targeting.association import accept_position, remembered_geometry
from planner.capabilities.completion.visual_goal_trigger import VisualGoalRadiusTrigger
from planner.domain.target_memory import TargetPositionMemory
from planner.domain.visual_goal import GoalGeometry
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.visual_goal_completion import VisualGoalTargetSnapshot, VisualGoalTriggerRules
from planner.domain.pose import WorldPose
from config.visual_goal import visual_goal_config
from tests.fakes.visual_goal import wall_observation, VISIBLE, INVISIBLE, Clock


def accept(memory, x, stamp):
    observation=wall_observation(x,stamp=stamp)
    return accept_position(memory,'stage',observation,VISIBLE,GoalGeometry(True,'pixel',x,(x,0.,-4.)),.5)


def test_first_valid_observation_locks_immediately_and_small_updates_remain_inside_first_anchor():
    memory,reason=accept(None,20.,1)
    assert reason is None and memory.anchor_world_point==memory.world_point
    for i,x in enumerate((20.2,20.4),2):
        memory,reason=accept(memory,x,i)
        assert reason is None and memory.anchor_world_point==(20.,0.,-4.)
    previous=memory
    memory,reason=accept(memory,20.6,4)
    assert reason=='outside_initial_target_update_radius' and memory is previous
    assert memory.timestamp_ns==3 and memory.world_point==(20.4,0.,-4.)
    memory,reason=accept(memory,49.,5)
    assert reason=='outside_initial_target_update_radius' and memory is previous


@pytest.mark.parametrize('invalid',['invisible','depth','old_frame'])
def test_rejection_keeps_original_source_identity_and_acquisition_time(invalid):
    memory,_=accept(None,20.,2)
    obs=wall_observation(20,stamp=1 if invalid=='old_frame' else 3)
    goal=INVISIBLE if invalid=='invisible' else VISIBLE
    geometry=GoalGeometry(False,'invalid depth') if invalid=='depth' else GoalGeometry(True,'pixel',20,(20.,0.,-4.))
    result,reason=accept_position(memory,'stage',obs,goal,geometry,.5)
    assert result is memory and reason
    with pytest.raises(ValueError,match='another stage'):
        accept_position(memory,'other',obs,goal,geometry,.5)


def test_remembered_target_stays_valid_for_navigation_but_does_not_refresh_its_timestamp():
    clock=Clock();memory,_=accept(None,20.,1)
    trigger=VisualGoalRadiusTrigger(VisualGoalTriggerRules(),1,clock)
    trigger.update(VisualGoalTargetSnapshot('stage',memory.world_point,memory.captured_monotonic,stage_memory=True))
    clock.sleep(60)
    assert not trigger.should_pause(WorldPose(0,0,-4,0))
    assert trigger.snapshot.captured_monotonic==100
    assert trigger.should_pause(WorldPose(17,0,-4,0))
    assert trigger.last_reason=='completion_radius'
    assert remembered_geometry(memory,WorldPose(17,0,-4,0)).distance_m==3


@pytest.mark.parametrize('kwargs',[{'timestamp_ns':0},{'world_point':(float('nan'),0,0)},
    {'world_point':[20,0,-4]},{'captured_monotonic':True},{'stage_id':''}])
def test_memory_requires_immutable_finite_scoped_evidence(kwargs):
    memory,_=accept(None,20.,1)
    with pytest.raises(ValueError):replace(memory,**kwargs)


@pytest.mark.parametrize('old', ['maximum_target_position_jump_m','target_center_margin'])
def test_removed_target_settings_report_a_migration(old):
    with pytest.raises(ValueError,match=old):visual_goal_config({'limits':{old:6}})


@pytest.mark.parametrize('value',[0,True,float('nan'),-1])
def test_update_radius_requires_positive_finite_meters(value):
    with pytest.raises(ValueError):VisualGoalLimits(target_position_update_radius_m=value)


def completion_request():
    from tests.unit.test_visual_goal_completion_client import request
    from planner.capabilities.targeting.association import remembered_goal
    req=request();obs=req.observation
    memory=TargetPositionMemory(req.stage_id,(3.5,0.,-4.),(3.5,0.,-4.),'initial',99.,1)
    return replace(req,target_memory=memory,goal=remembered_goal('bound car'),reference_rgb=obs.rgb.copy(),
                   geometry=remembered_geometry(memory,obs.vehicle_pose))


def test_memory_completion_needs_current_pose_and_reference_but_not_a_current_bbox():
    from planner.capabilities.completion.visual_goal_request import require_visual_completion_evidence
    from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
    req=completion_request()
    require_visual_completion_evidence(req)
    text=VisualGoalCompletionPrompt.build_user_prompt(req)
    assert '不是当前重新测得的目标表面距离' in text
    assert '尚待你判断' in text and '首次参考图' in text
    assert '本次关联目标框' not in text
    assert '已通过本地检查' not in text


@pytest.mark.parametrize('bad',['wrong_stage','wrong_distance','no_reference','moving','old_frame'])
def test_memory_completion_rejects_mismatched_or_unstopped_evidence(bad):
    from planner.capabilities.completion.visual_goal_request import require_visual_completion_evidence
    from planner.errors import ProtocolError
    req=completion_request()
    if bad=='wrong_stage':req=replace(req,stage_id='other')
    if bad=='wrong_distance':req=replace(req,geometry=replace(req.geometry,distance_m=0))
    if bad=='no_reference':req=replace(req,reference_rgb=None)
    if bad=='moving':req=replace(req,observation=replace(req.observation,stopped=False))
    if bad=='old_frame':req=replace(req,observation=replace(req.observation,captured_monotonic=98.))
    with pytest.raises(ProtocolError):require_visual_completion_evidence(req)


def refresh_fixture():
    from tests.integration.test_visual_goal_workflow import setup
    from tests.integration.test_visual_goal_radius_pause import Trace
    from planner.workflows.visual_goal_session import VisualGoalSession
    from planner.workflows.termination.visual_goal_navigation import VisualGoalExitPolicy, SearchPolicy
    from planner.domain.execution import ExecutionBudget
    from planner.capabilities.targeting.visual_goal_refresh import VisualGoalRefresh
    w,stage,source,observer,planner,vehicle,clock=setup(target_x=20)
    session=VisualGoalSession(ExecutionBudget(clock()+600),VisualGoalExitPolicy(w.limits),SearchPolicy(w.limits),clock(),
        initial=vehicle.pose,stage_id='stage')
    session.trigger=VisualGoalRadiusTrigger(w.trigger_rules,45,clock)
    tracker=VisualGoalRefresh(observer,w.geometry,w.limits,clock,w._event)
    trace=Trace()
    tracker.accept(wall_observation(20,pose=vehicle.pose,clock=clock),VISIBLE,session,trace)
    return w,stage,session,tracker,trace,clock


def test_detector_timeout_retains_memory_and_quarantines_late_result():
    import threading,time
    w,stage,session,tracker,trace,clock=refresh_fixture()
    entered,release=threading.Event(),threading.Event()
    original=session.target_memory
    def work():
        entered.set();release.wait(2);return VISIBLE
    try:
        tracker.job.start(work,.1);assert entered.wait(1)
        clock.sleep(.2)
        tracker.poll(session,trace)
        assert session.target_memory is original and tracker.job.busy
        assert tracker.job.future is None
        tracker.submit(wall_observation(49,stamp=2),stage.parameters,session,trace)
        assert tracker.job.future is None  # cannot replace the still-running request
        release.set()
        for _ in range(100):
            if not tracker.job.busy:break
            time.sleep(.001)
        tracker.poll(session,trace)
        assert session.target_memory is original
    finally:release.set();tracker.close();w.close()


@pytest.mark.parametrize('error',['BudgetExceeded','ExecutionError','ObservationError'])
def test_detector_wait_does_not_swallow_execution_or_observation_errors(error):
    import time
    import planner.errors as errors
    w,stage,session,tracker,trace,clock=refresh_fixture()
    def work():raise getattr(errors,error)('must propagate')
    try:
        tracker.job.start(work,1)
        for _ in range(100):
            if not tracker.job.busy:break
            time.sleep(.001)
        with pytest.raises(getattr(errors,error)):tracker.poll(session,trace)
    finally:tracker.close();w.close()
