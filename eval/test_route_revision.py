"""Focused tests for versioned queue rewrites used by local avoidance."""

import time

from agent.core.types import PlanOutput
from agent.functions.fast_slow.controller import FastSlowController
from agent.functions.trajectory_queue.world_queue import WorldTrajectoryQueue


def _wait_for_plan(controller, timeout_s=1.0):
    deadline = time.perf_counter() + timeout_s
    while controller.planning:
        if time.perf_counter() >= deadline:
            raise AssertionError("planner future did not finish")
        time.sleep(0.005)


def test_snapshot_suffix_replacement_tracks_consumed_prefix():
    queue = WorldTrajectoryQueue(
        world_waypoints=[
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
        ]
    )
    snapshot = queue.snapshot()

    queue.mark_executed(1)
    committed = queue.replace_suffix(
        [[2.0, -2.0, 0.0], [4.0, -2.0, 0.0]],
        start_index=2,
        base_revision=snapshot.revision,
        base_consumed_count=snapshot.consumed_count,
    )

    assert committed
    assert queue.world_waypoints == [
        [2.0, 0.0, 0.0],
        [2.0, -2.0, 0.0],
        [4.0, -2.0, 0.0],
    ]
    assert queue.route_revision == snapshot.revision + 1


def test_suffix_replacement_rejects_a_splice_point_already_passed():
    queue = WorldTrajectoryQueue(world_waypoints=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    snapshot = queue.snapshot()
    queue.mark_executed(2)

    assert not queue.replace_suffix(
        [[9.0, 0.0, 0.0]],
        start_index=1,
        base_revision=snapshot.revision,
        base_consumed_count=snapshot.consumed_count,
    )
    assert queue.world_waypoints == []


def test_normal_prefix_consumption_keeps_inflight_qwen_result_valid():
    queue = WorldTrajectoryQueue(world_waypoints=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    controller = FastSlowController({"MAX_PENDING": 5}, queue=queue)
    try:
        assert controller.maybe_submit_plan(
            current_pos=[0.0, 0.0, 0.0],
            current_yaw_deg=0.0,
            plan_fn=lambda _pending: PlanOutput(
                waypoints=[[1.0, 0.0, 0.0]],
                waypoint_format="incremental_body",
            ),
        )
        _wait_for_plan(controller)
        controller.mark_executed(1)

        assert controller.poll_plan() is not None
        assert queue.world_waypoints == [[2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    finally:
        controller.shutdown(wait=True)


def test_avoidance_rewrite_rejects_inflight_qwen_result():
    queue = WorldTrajectoryQueue(world_waypoints=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    controller = FastSlowController({"MAX_PENDING": 5}, queue=queue)
    try:
        assert controller.maybe_submit_plan(
            current_pos=[0.0, 0.0, 0.0],
            current_yaw_deg=0.0,
            plan_fn=lambda _pending: PlanOutput(
                waypoints=[[5.0, 0.0, 0.0]],
                waypoint_format="incremental_body",
            ),
        )
        _wait_for_plan(controller)
        revision = queue.route_revision
        assert queue.replace_world_waypoints([[1.0, -2.0, 0.0]], base_revision=revision)

        assert controller.poll_plan() is None
        assert queue.world_waypoints == [[1.0, -2.0, 0.0]]
    finally:
        controller.shutdown(wait=True)
