"""Real workflows and control loops with fake flight/observation only."""
from unittest.mock import Mock

import pytest

from tests.integration.test_basic_motion_workflow import build, stage
from tests.integration.test_motion_tracker import fake_airsim, run_motion
from tests.integration.test_visual_goal_workflow import setup
from tests.integration.test_visual_goal_window import enable_window
from planner.domain.pose import WorldPose


def test_basic_motion_default_does_not_read_collision_source():
    workflow, vehicle, _ = build()
    vehicle.collision_state = Mock(side_effect=AssertionError("collision read while disabled"))
    try:
        result = workflow.run(stage(forward_m=2))
        assert result.success, result.error
        assert vehicle.moves
        vehicle.collision_state.assert_not_called()
    finally:
        workflow.close()


@pytest.mark.parametrize("sliding", [False, True])
def test_visual_workflow_default_skips_collision_reads_through_planning_and_completion(sliding):
    workflow, mission_stage, _, _, planner, vehicle, _ = setup()
    vehicle.collision_state = Mock(side_effect=AssertionError("collision read while disabled"))
    if sliding:
        enable_window(workflow)
    try:
        result = workflow.run(mission_stage)
        assert result.success, result.error
        assert planner.calls and vehicle.segments
        vehicle.collision_state.assert_not_called()
    finally:
        workflow.close()


def test_visual_workflow_can_opt_in_and_reports_collision_failure():
    workflow, mission_stage, _, _, _, vehicle, _ = setup()
    read = Mock(return_value=True)
    workflow.collision_check = read
    try:
        result = workflow.run(mission_stage)
        assert not result.success and "collision" in result.error
        assert not vehicle.segments
        assert read.call_count == 1
        assert not workflow.execution.collision_state()  # Cleared on release.
    finally:
        workflow.close()


def test_motion_tracker_has_no_unconditional_collision_rpc(fake_airsim):
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(3, 0, -5, 45), collision_after=1)
    connection.call = Mock(side_effect=AssertionError("unexpected collision RPC"))
    tracker.execute(segment)
    assert connection.commands and connection.hovered
    connection.call.assert_not_called()
