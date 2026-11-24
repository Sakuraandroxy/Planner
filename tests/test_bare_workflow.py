"""The bare workflow: one detection, one model call, five waypoints, local radius."""

import numpy as np

from planner.domain.mission import MissionStage, TaskKind
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import (
    GoalGeometry,
    GoalObservation,
    GoalVisibility,
    VisualGoalNavigationParameters,
)
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.errors import ProtocolError
from planner.workflows.visual_goal_navigation.bare import BareVisualGoalNavigation

INSTRUCTION = "Drive forward and stop beside the red car."


class _Clock:
    """A deterministic clock; a zero step keeps the deadline out of reach."""

    def __init__(self, step=0.0):
        self.now = 0.0
        self.step = step

    def __call__(self):
        self.now += self.step
        return self.now


class _Source:
    def __init__(self, poses):
        self.poses = list(poses)
        self.captures = 0

    def capture(self, *, budget=None):
        pose = self.poses[min(self.captures, len(self.poses) - 1)]
        self.captures += 1
        return _observation(pose, capture_id=f"capture-{self.captures}",
                            timestamp_ns=self.captures * 1_000_000)


class _Observer:
    def __init__(self, status=GoalVisibility.VISIBLE):
        self.status = status
        self.calls = 0

    def observe(self, observation, target_description, reference=None, *,
                budget=None, initial_pose=None, instruction=None):
        self.calls += 1
        if self.status is not GoalVisibility.VISIBLE:
            return GoalObservation(self.status, None, None, target_description, "missing")
        return GoalObservation(self.status, (0.4, 0.4, 0.6, 0.6), None,
                               target_description, "visible")


class _Geometry:
    def __init__(self, world_point=(12.0, 0.0, 0.0)):
        self.world_point = world_point
        self.calls = 0

    def estimate(self, observation, goal):
        self.calls += 1
        if goal.status is not GoalVisibility.VISIBLE:
            return GoalGeometry(False, "target is not visible")
        return GoalGeometry(True, "test estimate", None, self.world_point,
                            surface_points_world=(self.world_point,))


class _Planner:
    def __init__(self, points):
        self.points = points
        self.calls = 0
        self.error = None

    def plan(self, observation, instruction, **kwargs):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return RelativeTrajectory(self.points)


class _PathVehicle:
    def __init__(self):
        self.calls = []
        self.error = None

    def follow_path(self, points, speed_mps, *, timeout_s=None):
        self.calls.append((tuple(points), speed_mps, timeout_s))
        if self.error is not None:
            raise self.error


class _Telemetry:
    def __init__(self):
        self.snapshots = []

    def publish_queue(self, **fields):
        self.snapshots.append(fields)


def _observation(pose: WorldPose, capture_id: str = "capture-0",
                 timestamp_ns: int = 1_000_000) -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.ones((480, 640), dtype=np.float32),
        depth_image=np.zeros((480, 640), dtype=np.uint8),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
        camera_id="front",
        timestamp_ns=timestamp_ns,
        capture_id=capture_id,
        captured_monotonic=1.0,
    )


def _stage() -> MissionStage:
    return MissionStage(
        "stage-1", TaskKind.VISUAL_GOAL_NAVIGATION,
        VisualGoalNavigationParameters(INSTRUCTION, "the red car"),
    )


def _seven_points() -> tuple[RelativePoseDelta, ...]:
    return tuple(RelativePoseDelta(0.8, 0.0, 0.0, 0.0) for _ in range(7))


def _workflow(*, poses, planner, observer=None, geometry=None, vehicle=None,
              limits=None, clock=None, telemetry=None):
    return BareVisualGoalNavigation(
        _Source(poses),
        observer or _Observer(),
        planner,
        vehicle or _PathVehicle(),
        geometry or _Geometry(),
        limits=limits or VisualGoalLimits(),
        clock=clock or _Clock(),
        telemetry=telemetry,
    )


def test_one_detection_one_plan_per_round_and_arrival_inside_the_radius():
    observer, telemetry = _Observer(), _Telemetry()
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=planner, observer=observer, vehicle=vehicle, telemetry=telemetry,
    )
    result = workflow.run(_stage())
    assert result.success is True
    assert result.summary.completion_confirmed is True
    assert result.summary.completion_basis.value == "local_measurement"
    assert observer.calls == 1
    assert planner.calls == 1
    assert len(vehicle.calls) == 1
    assert len(telemetry.snapshots) == 2


def test_more_than_five_waypoints_are_executed_as_the_first_five():
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=planner, vehicle=vehicle,
    )
    workflow.run(_stage())
    points, speed_mps, timeout_s = vehicle.calls[0]
    assert len(points) == 5
    assert speed_mps == 1.0
    assert timeout_s == 60.0
    assert abs(points[0].x - 0.8) < 1e-9
    assert abs(points[4].x - 4.0) < 1e-9


def test_fewer_than_five_waypoints_are_all_executed():
    vehicle = _PathVehicle()
    planner = _Planner((RelativePoseDelta(1.0, 0.0, 0.0, 0.0),) * 3)
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=planner, vehicle=vehicle,
    )
    workflow.run(_stage())
    assert len(vehicle.calls[0][0]) == 3


def test_arrival_check_uses_the_bound_surface_distance():
    planner, vehicle, telemetry = _Planner(_seven_points()), _PathVehicle(), _Telemetry()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(8.0, 0.0, 0.0, 0.0)),
        planner=planner, vehicle=vehicle, telemetry=telemetry,
    )
    result = workflow.run(_stage())
    assert result.success is True
    assert len(vehicle.calls) == 1
    assert telemetry.snapshots[-1]["items"] == ()


def test_a_target_that_is_never_bound_fails_with_that_reason():
    # The plan accepts blind motion until a limit fires, and the failure reason
    # must say that no target was ever bound.
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),),
        planner=planner, observer=_Observer(GoalVisibility.NOT_VISIBLE),
        vehicle=vehicle, clock=_Clock(step=100.0),
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "timeout"
    assert "no target was ever bound" in result.summary.reason
    assert result.value.final_distance_m is None
    assert len(vehicle.calls) >= 1


def test_a_parse_failure_ends_the_run_with_that_reason():
    planner = _Planner(())
    planner.error = ProtocolError("trajectory planner returned no JSON array")
    workflow = _workflow(poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner)
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "model_error"
    assert "trajectory parse failed" in result.summary.reason


def test_the_round_budget_ends_the_run():
    planner = _Planner(_seven_points())
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner,
        geometry=_Geometry(world_point=(100.0, 0.0, 0.0)),
        limits=VisualGoalLimits(maximum_planning_rounds=1),
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "timeout"
    assert planner.calls == 1
    assert "no target was ever bound" not in result.summary.reason


def test_an_engine_refusal_ends_the_run_with_that_reason():
    vehicle = _PathVehicle()
    vehicle.error = RuntimeError("engine refused the path")
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=_Planner(_seven_points()),
        vehicle=vehicle,
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "execution_error"
    assert "engine rejected or timed out" in result.summary.reason


def test_a_vehicle_without_path_following_is_rejected_before_the_run():
    workflow = _workflow(poses=(WorldPose(0.0, 0.0, 0.0, 0.0),),
                         planner=_Planner(_seven_points()), vehicle=object())
    try:
        workflow.prepare()
    except Exception as exc:
        assert "path-following vehicle" in str(exc)
    else:
        raise AssertionError("a vehicle without follow_path must be rejected")
