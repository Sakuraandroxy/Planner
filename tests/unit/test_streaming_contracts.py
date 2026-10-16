"""Regression cases for signed recovery, multi-batch pauses and image anchors."""
import json

import pytest

from planner.domain.pose import WorldPose, RelativePoseDelta
from planner.domain.streaming import TimedVehicleState
from planner.domain.trajectory import MotionSegment, MotionTrajectory, WorldTrajectory, RelativeTrajectory
from planner.capabilities.completion.visual_goal_retained_motion import retained_world_trajectory
from planner.services.trajectory_queue import WorldWaypointQueue
from planner.services.stream_timeline import StreamTimeline
from planner.services.target_coordinates import point_in_horizontal_body
from planner.adapters.trajectory_planner.prompts.visual_goal_navigation import VisualGoalTrajectoryPrompt
from tests.contract.test_visual_goal_planning_prompt import context
from tests.fakes.visual_goal import wall_observation
from planner.errors import ProtocolError


def batch(start=0, yaw=0):
    pose = WorldPose(float(start), 0, -2, yaw)
    return WorldTrajectory(pose, tuple(WorldPose(float(start+i), 0, -2, yaw) for i in range(1, 6)), (0.,)*5)


def test_real_log_small_yaw_drift_recovers_exact_original_endpoint():
    start = WorldPose(24.9696044921875, 3.456432342529297, -1.0381563901901245, 9.437282923171118)
    end = WorldPose(25.052092197068227, 3.4689882203476445, -1.0331632764358523, 9.437388000000027)
    actual = WorldPose(25.046560287475586, 3.4570000171661377, -1.0165694952011108, 9.437722098577616)
    motion = MotionTrajectory((MotionSegment(start, end, 'synchronized_quintic', 1, 2, .0001050768288983761),))
    world = retained_world_trajectory(motion, actual)
    assert world.poses == (end,)
    assert world.yaw_deltas_deg[0] == pytest.approx(end.yaw_deg-actual.yaw_deg, abs=1e-12)


def test_ten_authorized_points_survive_pause_without_becoming_a_ten_point_model_batch():
    q = WorldWaypointQueue(.3, 2)
    first = batch()
    q.load_batch(first)
    ids = tuple(p.waypoint_id for p in q.items)
    q.append_continuation(batch(5), ids, q.revision, WorldPose(.1, 0, -2, 0))
    state = TimedVehicleState(WorldPose(.5, 0, -2, 0), 100, 1.)
    paused = q.pause_stream(7, state, 0., 0., 'completion_radius')
    assert len(paused.remaining) == 10
    assert q.restore_stream(paused, 7) == 0.
    assert tuple(p.end.x for p in q.items) == tuple(range(1, 11))
    with pytest.raises(ProtocolError):
        q.restore_stream(paused, 8)


def test_passed_waypoint_does_not_require_zero_speed_but_requires_signed_yaw():
    q = WorldWaypointQueue(.3, 2)
    q.load_batch(batch())
    assert q.consume_stream(WorldPose(1.1, .1, -2, 0), 0., 0.)
    assert q.items[0].end.x == 2
    assert not q.consume_stream(WorldPose(2.1, .1, -2, 0), -180., 180.)


def test_simulator_time_selects_queue_before_or_after_pass_not_request_return_queue():
    q = WorldWaypointQueue(.3, 2)
    q.load_batch(batch())
    timeline = StreamTimeline()
    timeline.record(1_000_000_000, 1, q)
    q.consume_stream(WorldPose(1, 0, -2, 0), 0, 0)
    timeline.record(1_100_000_000, 1, q)
    assert len(timeline.at(1_050_000_000, 1).items) == 5
    assert len(timeline.at(1_150_000_000, 1).items) == 4
    with pytest.raises(ProtocolError):
        timeline.at(1_500_000_000, 1)


def test_relative_target_rotates_into_predicted_pending_tail_frame_and_omits_bbox():
    obs = wall_observation()
    pending = RelativeTrajectory((RelativePoseDelta(2, 0, 0, 90),))
    text = VisualGoalTrajectoryPrompt().build_user_prompt('approach', obs, 5,
        task_context=context(), pending_trajectory=pending)
    context_text = text.split('Visual goal context (estimated evidence, not target ground truth):\n', 1)[1]
    payload = json.loads(context_text.splitlines()[0])
    assert payload['target_relative_to_observation_body_m'] == [6., 0., 0.]
    assert payload['target_relative_to_pending_tail_body_m'] == pytest.approx([0, -4, 0])
    assert payload['new_actions_reference'] == 'pending_tail'
    assert 'bbox' not in payload and 'world_point' not in payload
    assert point_in_horizontal_body((1, -2, 3), WorldPose(0, 0, 1, 0)) == (1, -2, 2)
