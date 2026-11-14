import math

import numpy as np
import pytest

from planner.capabilities.planning import (
    GeometricPlannerSettings,
    GeometricTargetPlanner,
)
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    VisualGoalPlanningContext,
    VisualGoalState,
)
from planner.errors import ProtocolError


def _observation(pose: WorldPose) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.ones((480, 640), dtype=np.float32),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
    )


def _context(
    target: tuple[float, float, float],
    *,
    radius: float = 7.0,
    reliable: bool = True,
) -> VisualGoalPlanningContext:
    goal = GoalObservation(
        GoalVisibility.VISIBLE, None, None, "target", "manual"
    )
    geometry = GoalGeometry(
        reliable,
        "measured" if reliable else "unreliable",
        world_point=target if reliable else None,
    )
    return VisualGoalPlanningContext(
        "target",
        goal,
        geometry,
        VisualGoalState.APPROACHING,
        radius,
        0.0,
        None,
    )


def _plan(
    observation: Observation,
    context: VisualGoalPlanningContext,
    pending: RelativeTrajectory | None = None,
) -> RelativeTrajectory:
    return GeometricTargetPlanner().plan(
        observation,
        "go to the target",
        task_context=context,
        pending_trajectory=pending,
    )


def _advance(start: WorldPose, points) -> tuple[WorldPose, float]:
    pose = start
    length = 0.0
    for point in points:
        nxt = point.apply(pose)
        length += math.dist(
            (pose.x, pose.y, pose.z), (nxt.x, nxt.y, nxt.z)
        )
        pose = nxt
    return pose, length


def test_ahead_target_advances_to_buffer_distance():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    context = _context((10.0, 0.0, -15.0))
    trajectory = _plan(observation, context)
    assert len(trajectory.points) == 5
    assert all(point.dyaw_deg == 0.0 for point in trajectory.points)
    end, length = _advance(observation.pose, trajectory.points)
    # Buffer 0.8 * 7 m leaves 4.4 m to drive.
    assert end.x == pytest.approx(4.4, abs=1e-9)
    assert end.y == pytest.approx(0.0, abs=1e-9)
    assert length == pytest.approx(4.4, abs=1e-9)


def test_side_target_turns_and_moves_toward_it():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    context = _context((0.0, 10.0, -15.0))
    trajectory = _plan(observation, context)
    end, length = _advance(observation.pose, trajectory.points)
    assert end.x == pytest.approx(0.0, abs=1e-9)
    assert end.y == pytest.approx(4.4, abs=1e-9)
    assert end.yaw_deg == pytest.approx(90.0, abs=1e-9)
    assert length == pytest.approx(4.4, abs=1e-9)


def test_vertical_correction_is_clamped():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    context = _context((10.0, 0.0, -5.0))  # target 10 m below the vehicle
    end, _ = _advance(observation.pose, _plan(observation, context).points)
    assert end.z == pytest.approx(-15.0 + 2.0, abs=1e-9)


def test_waypoint_count_is_always_five():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    assert len(_plan(observation, _context((10.0, 0.0, -15.0))).points) == 5


def test_target_inside_buffer_produces_no_translation():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    trajectory = _plan(observation, _context((5.0, 0.0, -15.0)))
    for point in trajectory.points:
        assert point.dx == pytest.approx(0.0)
        assert point.dy == pytest.approx(0.0)


def test_pending_tail_is_the_planning_anchor():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    pending = RelativeTrajectory((RelativePoseDelta(dx=2.0),))
    context = _context((10.0, 0.0, -15.0))
    trajectory = _plan(observation, context, pending)
    tail, _ = _advance(observation.pose, pending.points)
    end, length = _advance(tail, trajectory.points)
    # From the pending tail at x=2 m, only 2.4 m remain before the buffer.
    assert end.x == pytest.approx(4.4, abs=1e-9)
    assert end.y == pytest.approx(0.0, abs=1e-9)
    assert length == pytest.approx(2.4, abs=1e-9)


def test_planning_requires_typed_context():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    with pytest.raises(ProtocolError):
        GeometricTargetPlanner().plan(observation, "go")


def test_planning_requires_reliable_geometry():
    observation = _observation(WorldPose(0.0, 0.0, -15.0, 0.0))
    context = _context((10.0, 0.0, -15.0), reliable=False)
    with pytest.raises(ProtocolError):
        _plan(observation, context)


def test_settings_validate():
    with pytest.raises(ValueError):
        GeometricPlannerSettings(max_advance_m=0.0)
    with pytest.raises(ValueError):
        GeometricPlannerSettings(arrival_buffer_ratio=1.0)


def test_zero_delta_has_no_motion():
    assert RelativePoseDelta().is_zero()