from dataclasses import replace
from types import SimpleNamespace

import pytest

from planner.adapters.airsim.observation_gate import ObservationGate
from planner.domain.execution import ExecutionBudget
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.domain.trajectory import WorldTrajectory
from planner.errors import BudgetExceeded, ExecutionError
from planner.services.execution import ExecutionService
from planner.services.motion import SynchronizedMotionPlanner
from planner.workflows.termination.visual_goal_navigation import SearchPolicy
from planner.domain.visual_goal_limits import VisualGoalLimits
from tests.fakes.components import FakeVehicle
from tests.fakes.visual_goal import Clock


def test_expired_execution_budget_never_sends_motion_and_attempts_both_stop_actions():
    vehicle, clock = FakeVehicle(), Clock()
    motion = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(
        WorldTrajectory(vehicle.pose, (WorldPose(1, 0, 0, 0),)))
    with pytest.raises(BudgetExceeded) as raised:
        ExecutionService(vehicle, clock).execute_with_receipt(motion, budget=ExecutionBudget(clock()))
    assert not vehicle.segments and vehicle.cancelled and vehicle.hovered
    assert raised.value.receipt.completed_segments == 0
    assert not raised.value.stop_result.confirmed


def test_measured_chords_do_not_collapse_out_and_back_into_zero_length():
    vehicle = FakeVehicle()
    motion = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(
        WorldTrajectory(vehicle.pose, (WorldPose(1, 0, 0, 0), WorldPose(0, 0, 0, 0))))
    receipt = ExecutionService(vehicle).execute_with_receipt(motion)
    assert receipt.measured_chord_length_m == pytest.approx(2)


def test_observation_failure_poisoning_blocks_future_capture_and_motion():
    clock = Clock()
    vehicle = SimpleNamespace(hover=lambda: None, current_pose=lambda: WorldPose(0, 0, 0, 0),
                              collision_state=lambda: False)
    gate = ObservationGate(vehicle, MotionLimits(), 2, clock, clock.sleep)
    with pytest.raises(RuntimeError):
        with gate.capture():
            raise RuntimeError("camera disconnected")
    with pytest.raises(ExecutionError, match="restart"):
        with gate.capture():
            pass
    with pytest.raises(ExecutionError, match="restart"):
        with gate.step():
            pass


def test_observation_stabilization_obeys_task_budget():
    clock = Clock()
    vehicle = SimpleNamespace(hover=lambda: None, current_pose=lambda: WorldPose(0, 0, 0, 0),
                              collision_state=lambda: False)
    gate = ObservationGate(vehicle, MotionLimits(), 20, clock, clock.sleep)
    with pytest.raises(BudgetExceeded):
        with gate.capture(budget=ExecutionBudget(clock()+0.1)):
            pytest.fail("unstabilized observation must not be produced")


def test_search_prioritizes_explicit_right_hint_with_small_incremental_turns():
    search = SearchPolicy(replace(VisualGoalLimits(), search_clearance_confirmed=True))
    pose = WorldPose(0, 0, 0, 0)
    assert search.next_heading(pose, direction_hint="white car on the right") == 30
