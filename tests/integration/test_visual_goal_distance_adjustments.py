"""Lateral adjustments may precede approach; use synthetic flight and model I/O."""
from dataclasses import replace
import math

import pytest

from planner.domain.pose import RelativePoseDelta
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import VisualGoalStatus
from planner.services.trajectory_transform import relative_to_world
from tests.fakes.visual_goal import INVISIBLE
from tests.integration.test_continuous_visual_goal import continuous_setup
from tests.integration.test_visual_goal_workflow import setup


def lateral_then_approach(workflow, planner, target_x, lateral_batches=6):
    original_plan = planner.plan

    def plan(observation, instruction, **kwargs):
        original_plan(observation, instruction, **kwargs)
        if len(planner.calls) <= lateral_batches:
            return RelativeTrajectory((RelativePoseDelta(0, 1, 0, 0),)*5)
        pending = kwargs.get("pending_trajectory")
        start = (relative_to_world(pending, observation.vehicle_pose).poses[-1]
                 if pending is not None and pending.points else observation.vehicle_pose)
        return RelativeTrajectory((RelativePoseDelta((target_x-start.x)/5, -start.y/5, 0, 0),)*5)

    planner.plan = plan
    # Only the initial detection is supplied; later flight uses the locked point.
    observer = workflow.goal_observer
    original_observe = observer.observe

    def observe(observation, description, reference=None, **kwargs):
        if reference is not None:
            return INVISIBLE
        return original_observe(observation, description, reference, **kwargs)

    observer.observe = observe


@pytest.mark.parametrize("mode", ["serial", "continuous"])
def test_multiple_lateral_adjustments_continue_then_complete_at_locked_target(mode):
    if mode == "continuous":
        workflow, stage, _, planner, _, _ = continuous_setup(target_x=30)
    else:
        workflow, stage, _, _, planner, _, _ = setup(target_x=30)
    lateral_then_approach(workflow, planner, 30)
    try:
        result = workflow.run(stage)
        assert result.success, result.error
        assert len(planner.calls) >= 7
        # These distinct observations show real lateral motion without reducing
        # distance to the fixed target, before the later approach succeeds.
        positions = [call[0].vehicle_pose for call in planner.calls[:6]]
        assert positions[-1].y-positions[0].y > 15
        target = (30, 0, positions[0].z)
        distances = [math.dist((p.x, p.y, p.z), target) for p in positions]
        assert min(distances[1:]) >= distances[0]-.01
        assert result.value.completion_confirmed
        assert not result.value.remaining_motion.segments
    finally:
        workflow.close()


def test_continuous_adjustments_still_end_at_existing_stage_deadline():
    workflow, stage, vehicle, planner, _, _ = continuous_setup(target_x=30)
    workflow.limits = replace(workflow.limits, maximum_task_duration_s=8)
    lateral_then_approach(workflow, planner, 30, lateral_batches=100)
    try:
        result = workflow.run(stage)
        assert not result.success and result.value.status is VisualGoalStatus.TIMEOUT
        assert result.value.elapsed_s >= 8
        assert vehicle.commands and vehicle.hovers
        assert not result.value.completion_confirmed
    finally:
        workflow.close()
