"""Real refill worker + owner flight fakes; no RPCs, GPU or model requests."""
import threading
from dataclasses import replace

import pytest

from planner.domain.motion import MotionLimits
from planner.domain.pose import RelativePoseDelta
from planner.domain.sliding_window import SlidingWindowSettings
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import VisualGoalStatus
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.services.planning_window import PlanningWindow
from tests.fakes.components import FakeVehicle
from tests.integration.test_visual_goal_workflow import setup
from tests.integration.test_visual_goal_radius_pause import partial_setup, Trace


class ControlledPlanner:
    def __init__(self, points, block=True):
        self.points, self.block = points, block
        self.calls = []
        self.entered, self.release, self.finished = threading.Event(), threading.Event(), threading.Event()

    def plan(self, observation, instruction, **context):
        self.calls.append((threading.get_ident(), observation, instruction, context))
        assert not observation.depth_meters.flags.writeable
        if context["pending_trajectory"].points and self.block:
            self.entered.set()
            assert self.release.wait(2), "test did not release the one pending model call"
            self.finished.set()
        return RelativeTrajectory(self.points)


def enable_window(workflow, planner=None):
    workflow.window = PlanningWindow(planner or workflow.planner, workflow.validator,
        SlidingWindowSettings(enabled=True, latency_margin_s=0.01), MotionLimits(), workflow.clock)
    return workflow.window


def test_slow_refill_overlaps_owner_flight_and_continues_from_pending_tail():
    limits = replace(VisualGoalLimits())
    workflow, stage, source, observer, _, _, clock = setup(target_x=10, limits=limits)
    stage = replace(stage, parameters=replace(stage.parameters,
        instruction="Approach the red house. 接近红色房子。"))
    planner = ControlledPlanner(tuple(RelativePoseDelta(0.6, 0, 0, 0) for _ in range(5)))
    owner = threading.get_ident()

    class Vehicle(FakeVehicle):
        def execute_segment(self, segment, **kwargs):
            assert threading.get_ident() == owner
            if len(self.segments) == 2:
                assert planner.entered.wait(1)
                assert not planner.release.is_set()
            super().execute_segment(segment, **kwargs)
            if len(self.segments) == 5:
                planner.release.set()

    vehicle = Vehicle()
    source.vehicle = workflow.execution.vehicle = vehicle
    window = enable_window(workflow, planner)
    try:
        result = workflow.run(stage)
        assert result.success, result.error
        pending_calls = [call for call in planner.calls if call[3]["pending_trajectory"].points]
        assert pending_calls and planner.finished.is_set()
        first = pending_calls[0]
        assert first[0] != owner and first[1].vehicle_pose.x == pytest.approx(1.2)
        assert len(first[3]["pending_trajectory"].points) == 3
        assert len(first[3]["previous_trajectory"].points) == 2
        assert first[3]["previous_trajectory_start"].x == 0
        assert first[3]["task_context"].goal_observation_is_current is False
        assert first[3]["task_context"].goal.bbox is None
        assert "接近红色房子" in first[2]
        assert sum(not call[3]["pending_trajectory"].points for call in planner.calls) == 1
        assert [segment.end.x for segment in vehicle.segments] == pytest.approx(
            [0.6*index for index in range(1, len(vehicle.segments)+1)])
        assert not window.queue.items and window.background.candidate is None
        assert window.background.job is None and result.value.completion_confirmed
    finally:
        planner.release.set()
        workflow.close()


@pytest.mark.parametrize("answers", [(True,), (False, True)])
def test_radius_pause_preserves_queue_until_true_and_false_resumes_world_goal(answers):
    workflow, stage, source, planner, vehicle, checker, trace = partial_setup(answers)
    window = enable_window(workflow)
    try:
        result = workflow.run(stage)
        assert result.success, result.error
        pauses = [data for name, data in trace.events if name == "radius_pause"]
        assert pauses and pauses[0]["remaining_points"] > 0
        checks = [data for name, data in trace.events if name == "completion_check"]
        assert [item["completed"] for item in checks] == list(answers)
        assert checks[0]["retained_points"] > 0
        assert all(segment.end.x == 2 for segment in vehicle.segments)
        assert vehicle.pose.x <= 2.01
        assert not result.value.remaining_motion.segments and not window.queue.items
        # An early radius pause can occur before the five-point queue needs refill.
        assert pauses[0]["remaining_points"] == 5
        assert sum(not call[2]["pending_trajectory"].points for call in planner.calls) == 1
        assert window.background.job is None and window.background.candidate is None
    finally:
        workflow.close()


def test_window_timeout_keeps_pending_and_refuses_next_stage_until_old_call_returns():
    workflow, stage, source, _, _, _, clock = setup(target_x=10,
        limits=replace(VisualGoalLimits(), model_request_timeout_s=0.05))
    planner = ControlledPlanner(tuple(RelativePoseDelta(2, 0, 0, 0) for _ in range(5)))

    class Vehicle(FakeVehicle):
        def execute_segment(self, segment, **kwargs):
            if len(self.segments) >= 2:
                assert planner.entered.wait(1)
                # The refill starts with three points pending; radius pause retains them.
                clock.sleep(0.1)
            super().execute_segment(segment, **kwargs)

    vehicle = Vehicle()
    source.vehicle = workflow.execution.vehicle = vehicle
    window = enable_window(workflow, planner)
    try:
        result = workflow.run(stage)
        assert not result.success and result.value.status is VisualGoalStatus.TIMEOUT
        assert result.value.remaining_motion.segments
        assert window.worker.has_pending_call
        captures = source.captures
        next_result = workflow.run(replace(stage, stage_id="next"))
        assert not next_result.success and source.captures == captures
        assert len(planner.calls) == 2
    finally:
        planner.release.set()
        planner.finished.wait(1)
        workflow.close()


def test_completed_window_can_be_reused_for_another_stage_with_empty_initial_pending():
    workflow, stage, source, observer, planner, vehicle, clock = setup(target_x=6)
    trace = Trace()
    workflow.trace_factory = lambda: trace
    window = enable_window(workflow)
    try:
        first = workflow.run(stage)
        assert first.success, first.error
        call_count = len(planner.calls)
        source.target_x = vehicle.pose.x+6
        second = workflow.run(replace(stage, stage_id="second"))
        assert second.success, second.error
        assert planner.calls[call_count][2]["pending_trajectory"].points == ()
        assert planner.calls[call_count][2]["round_index"] == 1
        assert not window.queue.items
    finally:
        workflow.close()


def test_valid_speculative_points_execute_without_corridor_review():
    limits = replace(VisualGoalLimits(), maximum_planning_rounds=2)
    workflow, stage, source, observer, _, vehicle, clock = setup(target_x=10, limits=limits)
    trace = Trace()
    workflow.trace_factory = lambda: trace
    calls = []

    class Planner:
        def plan(self, observation, instruction, **context):
            calls.append(context)
            point = RelativePoseDelta(.6, 3, 0, 0) if context["pending_trajectory"].points else RelativePoseDelta(.6, 0, 0, 0)
            return RelativeTrajectory((point,)*5)

    enable_window(workflow, Planner())
    try:
        result = workflow.run(stage)
        assert result.value.status is VisualGoalStatus.TIMEOUT  # The valid route misses the fixed memory point.
        assert any(segment.end.y > 0 for segment in vehicle.segments)
        assert sum(not call["pending_trajectory"].points for call in calls) == 1
        assert all(data["execution_policy"] == "direct_model_trajectory" for name, data in trace.events if name == "plan")
    finally:
        workflow.close()


def test_window_rejects_short_model_reply_without_moving():
    workflow, stage, source, observer, _, vehicle, clock = setup(target_x=10)
    planner = ControlledPlanner((RelativePoseDelta(.6, 0, 0, 0),)*4, block=False)
    enable_window(workflow, planner)
    try:
        result = workflow.run(stage)
        assert not result.success and result.value.status is VisualGoalStatus.MODEL_ERROR
        assert not vehicle.segments and len(planner.calls) == 2
    finally:
        workflow.close()
