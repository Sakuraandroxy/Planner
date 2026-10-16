"""Continuous owner flight with real background scheduling and synthetic sensors."""
import math
import time
import threading
from dataclasses import replace

import pytest

from planner.domain.pose import WorldPose, RelativePoseDelta
from planner.domain.streaming import TimedVehicleState
from planner.domain.sliding_window import SlidingWindowSettings
from planner.domain.motion import MotionLimits
from planner.domain.trajectory import WorldTrajectory, RelativeTrajectory
from planner.services.continuous_execution import ContinuousExecution
from planner.services.execution import ExecutionService
from planner.services.planning_window import PlanningWindow
from planner.services.trajectory_queue import WorldWaypointQueue
from planner.workflows.visual_goal_continuous import VisualGoalContinuous
from tests.fakes.components import FakeVehicle
from tests.fakes.visual_goal import wall_observation, CompletionChecker
from tests.integration.test_visual_goal_workflow import setup
from tests.integration.test_visual_goal_radius_pause import Trace


class StreamVehicle(FakeVehicle):
    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.velocity = (0., 0., 0.)
        self.commands, self.hovers = [], 0
        self.hover_positions = []

    def stream_state(self):
        return TimedVehicleState(self.pose, round(self.clock()*1e9), self.clock(), self.velocity)

    def stream_velocity(self, velocity, rate, duration):
        self.commands.append((self.clock(), self.pose, velocity, rate))
        self.velocity = velocity
        self.pose = WorldPose(self.pose.x+velocity[0]*duration, self.pose.y+velocity[1]*duration,
            self.pose.z+velocity[2]*duration, (self.pose.yaw_deg+rate*duration+180)%360-180)
        self.clock.sleep(duration)
        time.sleep(.0001)  # let data-only workers run; no wall-clock model waits

    def hover(self):
        self.hovers += 1
        self.hover_positions.append(self.pose)
        self.velocity = (0., 0., 0.)


class Source:
    def __init__(self, vehicle, clock, target_x):
        self.vehicle, self.clock, self.target_x = vehicle, clock, target_x
        self.counter, self.closed = 0, False

    def capture(self, *, budget=None):
        self.counter += 1
        return wall_observation(self.target_x-self.vehicle.pose.x, self.vehicle.pose, clock=self.clock,
            stamp=round(self.clock()*1e9)+self.counter)

    def capture_moving(self, *, budget=None):
        return replace(self.capture(budget=budget), stopped=False)

    def close(self):
        self.closed = True


def continuous_setup(target_x=30, answers=(True,)):
    workflow, stage, _, observer, planner, _, clock = setup(target_x=target_x)
    vehicle = StreamVehicle(clock)
    source = Source(vehicle, clock, target_x)
    workflow.source = source
    workflow.execution.vehicle = vehicle
    workflow.verification.checker = CompletionChecker(answers)
    # Verification stores the checker directly; configure through its public
    # constructor so the fake matches production's independent model role.
    from planner.capabilities.completion.visual_goal_verification import VisualGoalVerification
    workflow.verification = VisualGoalVerification(CompletionChecker(answers), clock)
    workflow.flight.verification = workflow.verification
    workflow.window = PlanningWindow(planner, workflow.validator, SlidingWindowSettings(enabled=True), MotionLimits(), clock)
    def sleep(seconds):
        clock.sleep(seconds)
        time.sleep(.0001)  # Let data-only workers run even while the queue is empty.
    workflow.continuous = VisualGoalContinuous(workflow.execution, source, workflow.limits, MotionLimits(), 2.,
        observer, workflow.geometry, workflow.completion, workflow.flight, workflow._capture,
        workflow._search, clock, sleep, workflow._event)
    trace = Trace()
    workflow.trace_factory = lambda: trace
    return workflow, stage, vehicle, planner, trace, clock


def test_two_batches_append_without_stopping_at_intermediate_points_and_use_five_second_clock():
    w, stage, vehicle, planner, trace, clock = continuous_setup()
    try:
        result = w.run(stage)
        assert result.success, result.error
        calls = [d for n,d in trace.events if n=='planning_submitted']
        appends = [d for n,d in trace.events if n=='queue_appended']
        assert len(calls) >= 2 and len(appends) >= 2
        assert any(d['queue_length'] > 5 for d in appends)
        assert all(d['planning_interval_s'] == 5 for d in calls)
        schedules=[d for n,d in trace.events if n=='planning_schedule']
        assert all(d['elapsed_since_previous_s'] >= 5 for d in schedules[1:])
        assert not [d for n,d in trace.events if n=='queue_exhausted']
        passed = [d for n,d in trace.events if n=='waypoint_passed']
        assert len(passed) >= 6
        assert all(math.hypot(*d['velocity']) > .15 for d in passed)
        pauses = [d for n,d in trace.events if n=='pause_requested']
        assert vehicle.hovers == len(pauses)
        assert all(p.x >= 25-.01 for p in vehicle.hover_positions)
        assert not w.window.queue.items
    finally:
        w.close()


def test_false_completion_resumes_retained_world_points_and_true_clears_them():
    w, stage, vehicle, planner, trace, clock = continuous_setup(target_x=14, answers=(False,True))
    try:
        result = w.run(stage)
        assert result.success, result.error
        checks = [d for n,d in trace.events if n=='completion_check']
        assert [d['completed'] for d in checks] == [False,True]
        assert all(d['retained_points'] > 0 for d in checks)
        assert checks[0]['retained_points'] > 5
        assert not w.window.queue.items and not result.value.remaining_motion.segments
        assert all(p.x >= 9-.01 for p in vehicle.hover_positions)
    finally:
        w.close()


@pytest.mark.parametrize('turn',[180.,-180.])
def test_continuous_pure_rotation_preserves_signed_half_turn(turn):
    from tests.fakes.visual_goal import Clock
    clock = Clock(); vehicle = StreamVehicle(clock)
    q = WorldWaypointQueue(.3,2)
    start = vehicle.pose
    end = replace(start, yaw_deg=-180.)
    q.load_prefix(WorldTrajectory(start,(end,),(turn,)))
    execution=ExecutionService(vehicle,clock)
    stream=ContinuousExecution(execution,vehicle,MotionLimits(),2,clock)
    stream.begin(q,1)
    for _ in range(800):
        stream.tick()
        if not q.items:break
    assert not q.items
    rates=[x[3] for x in vehicle.commands]
    assert (max(rates)>0 and min(rates)>=0) if turn>0 else (min(rates)<0 and max(rates)<=0)
    assert vehicle.pose.x==start.x and vehicle.pose.y==start.y


def test_slow_second_request_keeps_flying_with_one_request_in_flight():
    w,stage,vehicle,planner,trace,clock=continuous_setup()
    entered,release=threading.Event(),threading.Event()
    original_plan,original_move=planner.plan,vehicle.stream_velocity
    while_waiting=[]
    owner=threading.get_ident()
    image_requests=[]
    def record_images(request_id,observation):
        assert threading.get_ident()!=owner
        image_requests.append(request_id)
    trace.planning_observation=record_images
    def plan(*args,**kwargs):
        if len(planner.calls)==1:
            entered.set()
            assert release.wait(2), 'control loop failed to keep moving during model wait'
        return original_plan(*args,**kwargs)
    def move(*args):
        original_move(*args)
        if entered.is_set() and not release.is_set():
            while_waiting.append(vehicle.pose)
            if len(while_waiting)==5:release.set()
    planner.plan,vehicle.stream_velocity=plan,move
    try:
        result=w.run(stage)
        assert result.success,result.error
        assert len(while_waiting)==5
        assert while_waiting[-1].x>while_waiting[0].x
        assert len(image_requests)>=2
        assert not [d for n,d in trace.events if n=='queue_exhausted']
    finally:release.set();w.close()


def test_queue_exhaustion_hovers_then_accepts_late_continuation_without_rebasing():
    w,stage,vehicle,planner,trace,clock=continuous_setup()
    entered,release=threading.Event(),threading.Event()
    original_plan,original_hover=planner.plan,vehicle.hover
    def plan(*args,**kwargs):
        if len(planner.calls)==1:
            entered.set()
            assert release.wait(2), 'queue never reached its waiting boundary'
        return original_plan(*args,**kwargs)
    def hover():
        original_hover()
        if entered.is_set() and not release.is_set():
            release.set()
            time.sleep(.002)
    planner.plan,vehicle.hover=plan,hover
    try:
        result=w.run(stage)
        assert result.success,result.error
        assert [d for n,d in trace.events if n=='queue_exhausted']
        assert vehicle.hover_positions[0].x==pytest.approx(10,abs=.3)
        assert not [d for n,d in trace.events if n=='planning_discarded']
    finally:release.set();w.close()


def test_control_failure_stops_and_reports_unfinished_points_then_closes_read_source():
    from planner.errors import ExecutionError
    w,stage,vehicle,planner,trace,clock=continuous_setup()
    source=w.continuous.frames.source
    original_move=vehicle.stream_velocity
    def move(*args):
        if len(vehicle.commands)>=3:raise ExecutionError('synthetic controller failure')
        original_move(*args)
    vehicle.stream_velocity=move
    try:
        result=w.run(stage)
        assert not result.success and result.value.status.value=='execution_error'
        assert result.value.remaining_motion.segments
        assert vehicle.hovers>=1
    finally:w.close()
    assert source.closed


def test_timed_out_qwen_request_cannot_reach_next_stage_or_replace_its_queue():
    from planner.errors import ExecutionError
    w,stage,vehicle,planner,trace,clock=continuous_setup()
    release=threading.Event()
    original_plan=planner.plan
    request_frames=w.continuous.frames.request
    # Isolate Qwen's timeout from the separately tested capture timeout.
    w.continuous.frames.request=lambda budget:None
    w.continuous.sleep=lambda seconds:(clock.sleep(seconds),time.sleep(.0001))
    def plan(*args,**kwargs):
        # Advance beyond this request's deadline, but keep the HTTP-equivalent
        # call running so the next stage must refuse handover.
        clock.sleep(kwargs['timeout_s']+.01)
        release.wait(2)
        return original_plan(*args,**kwargs)
    planner.plan=plan
    try:
        result=w.run(stage)
        assert not result.success and result.value.status.value=='timeout'
        assert w.has_pending_calls()
        with pytest.raises(ExecutionError):w.prepare()
        assert not w.window.queue.items and not vehicle.commands
        release.set()
        for _ in range(100):
            if not w.has_pending_calls():break
            time.sleep(.001)
        w.prepare()
        assert not w.window.queue.items
        # A subsequent stage at the same static target starts with fresh
        # evidence and an empty queue; the old response remains discarded.
        planner.plan=original_plan
        w.continuous.frames.request=request_frames
        again=w.run(replace(stage,stage_id='goal_after_timeout'))
        assert again.success,again.error
    finally:release.set();w.close()


def test_success_drains_discarded_request_before_mission_can_start_next_stage():
    w,stage,vehicle,planner,trace,clock=continuous_setup(target_x=14)
    entered,release=threading.Event(),threading.Event()
    original_plan=planner.plan
    original_check=w.verification.checker.check
    def plan(*args,**kwargs):
        if len(planner.calls)==1:
            entered.set()
            assert release.wait(2)
            time.sleep(.003)  # still finishing when completion returns true
        return original_plan(*args,**kwargs)
    def check(*args,**kwargs):
        assert entered.is_set()
        release.set()
        return original_check(*args,**kwargs)
    planner.plan=plan
    w.verification.checker.check=check
    w.continuous.sleep=lambda seconds:(clock.sleep(seconds),time.sleep(.0001))
    try:
        result=w.run(stage)
        assert result.success,result.error
        assert not w.has_pending_calls() and not w.window.queue.items
        planner.plan=original_plan
        w.verification.checker.check=original_check
        again=w.run(replace(stage,stage_id='immediate_next_stage'))
        assert again.success,again.error
    finally:release.set();w.close()
