"""Quarantine and object-identity checks are separate from center-depth sampling."""
from dataclasses import replace
import threading
import time

import pytest

from planner.capabilities.targeting.visual_goal_refresh import VisualGoalRefresh
from planner.services.background_operation import BackgroundOperation
from planner.errors import ExecutionError, ModelCallTimeout
from planner.domain.pose import WorldPose
from planner.capabilities.completion.visual_goal_trigger import VisualGoalRadiusTrigger
from tests.fakes.visual_goal import Clock, wall_observation, VISIBLE
from tests.integration.test_visual_goal_workflow import setup
from tests.integration.test_visual_goal_radius_pause import Trace
from planner.workflows.visual_goal_session import VisualGoalSession
from planner.workflows.termination.visual_goal_navigation import VisualGoalExitPolicy, SearchPolicy
from planner.domain.execution import ExecutionBudget


def test_timed_out_operation_does_not_allow_another_thread_or_collect_late_result():
    clock=Clock();job=BackgroundOperation(clock)
    entered,release,finished=threading.Event(),threading.Event(),threading.Event()
    def work():
        entered.set();release.wait(2);finished.set();return 'late'
    try:
        job.start(work,.1);assert entered.wait(1)
        clock.sleep(.2)
        with pytest.raises(ModelCallTimeout):job.poll()
        assert job.busy
        with pytest.raises(ExecutionError):job.start(lambda:'new',1)
        release.set();assert finished.wait(1)
        for _ in range(100):
            if not job.busy:break
            time.sleep(.001)
        assert job.poll() is None
        job.start(lambda:'fresh',1)
        for _ in range(100):
            if not job.busy:break
            time.sleep(.001)
        assert job.poll()=='fresh'
    finally:release.set();job.close()


def test_wrong_car_position_does_not_replace_bound_target_or_its_refresh_time():
    w,stage,source,observer,planner,vehicle,clock=setup(target_x=25)
    limits=w.limits
    session=VisualGoalSession(ExecutionBudget(clock()+600),VisualGoalExitPolicy(limits),SearchPolicy(limits),clock(),
        initial=vehicle.pose,stage_id='goal')
    session.trigger=VisualGoalRadiusTrigger(w.trigger_rules,45,clock)
    tracker=VisualGoalRefresh(observer,w.geometry,limits,clock,w._event)
    trace=Trace()
    try:
        first=wall_observation(25,pose=vehicle.pose,clock=clock,stamp=100)
        assert tracker.accept(first,VISIBLE,session,trace)
        reference=session.reference
        timestamp=session.trigger.snapshot.captured_monotonic
        clock.sleep(1)
        wrong=wall_observation(50,pose=vehicle.pose,clock=clock,stamp=200)
        assert not tracker.accept(wrong,VISIBLE,session,trace)
        assert session.reference is reference
        assert session.trigger.snapshot.captured_monotonic==timestamp
        assert tracker.failures==1
        assert any(n=='target_update_rejected' and d['reason']=='outside_initial_target_update_radius'
                   for n,d in trace.events)
        assert session.target_memory.world_point == reference.world_point
    finally:tracker.close();w.close()
