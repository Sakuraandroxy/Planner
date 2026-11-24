"""The bare workflow: one detection, one model call, five waypoints, local radius."""

import time

import numpy as np
import pytest

from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.prompts import BareTrajectoryPrompt
from planner.adapters.trajectory_planner.response_parser import (
    parse_translation_trajectory,
)
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
from planner.domain.visual_goal_limits import BareNavigationSettings, VisualGoalLimits
from planner.errors import ExecutionError, ProtocolError
from planner.workflows.visual_goal_navigation.bare import BareVisualGoalNavigation

INSTRUCTION = "Drive forward and stop beside the red car."


class _Clock:
    """A deterministic clock that keeps wall-clock deadlines reachable.

    ``QwenVLPlanner`` compares the injected budget with ``time.monotonic()``, so
    the base value has to sit near real time; ``step`` only drives timeouts.
    """

    def __init__(self, step=0.0):
        self.now = time.monotonic()
        self.step = step

    def __call__(self):
        self.now += self.step
        return self.now

    def expire(self):
        """Push the clock past any budget, as if the engine call took too long."""
        self.now += 10_000.0


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
    """Detector fake; ``statuses`` gives a different answer per round."""

    def __init__(self, status=GoalVisibility.VISIBLE, failures=0, statuses=None):
        self.status = status
        self.failures = failures
        self.statuses = tuple(statuses) if statuses is not None else None
        self.calls = 0

    def observe(self, observation, target_description, reference=None, *,
                budget=None, initial_pose=None, instruction=None):
        self.calls += 1
        if self.calls <= self.failures:
            raise ProtocolError("detector request failed")
        status = self.status
        if self.statuses:
            status = self.statuses[min(self.calls - 1, len(self.statuses) - 1)]
        if status is not GoalVisibility.VISIBLE:
            return GoalObservation(status, None, None, target_description, "missing")
        return GoalObservation(status, (0.4, 0.4, 0.6, 0.6), None,
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
    def __init__(self, points, failures=0):
        self.points = points
        self.failures = failures
        self.calls = 0
        self.round_indices = []
        self.error = None

    def plan(self, observation, instruction, **kwargs):
        self.calls += 1
        self.round_indices.append(kwargs.get("round_index"))
        if self.calls <= self.failures:
            raise ProtocolError("planner request failed")
        if self.error is not None:
            raise self.error
        return RelativeTrajectory(self.points)


class _PathVehicle:
    def __init__(self, events=None):
        self.calls = []
        self.error = None
        self.events = events

    def follow_path(self, points, speed_mps, *, timeout_s=None):
        if self.events is not None:
            self.events.append("follow")
        self.calls.append((tuple(points), speed_mps, timeout_s))
        if self.error is not None:
            raise self.error


class _FollowThenExpire(_PathVehicle):
    """Returns from the engine call after the stage time budget has blown."""

    def __init__(self, clock):
        super().__init__()
        self.clock = clock

    def follow_path(self, points, speed_mps, *, timeout_s=None):
        super().follow_path(points, speed_mps, timeout_s=timeout_s)
        self.clock.expire()


class _Telemetry:
    def __init__(self, events=None):
        self.snapshots = []
        self.events = events

    def publish_queue(self, **fields):
        if self.events is not None:
            self.events.append("publish")
        self.snapshots.append(fields)


class _Execution:
    """Stands in for ``ExecutionService``: only the checkpoint is read."""

    def __init__(self):
        self.checkpoints = 0

    def checkpoint(self):
        self.checkpoints += 1


class _StopAfter(_Execution):
    """Raises the flight-stop error once the source has captured enough frames.

    Model calls also run a checkpoint per poll, so the counter is the capture
    count, not the checkpoint count.
    """

    def __init__(self, source, captures):
        super().__init__()
        self.source = source
        self.captures = captures

    def checkpoint(self):
        super().checkpoint()
        if self.source.captures >= self.captures:
            raise ExecutionError("flight stop requested")


class _Transport:
    """Records the prompts the planner sends and replays one reply."""

    def __init__(self, reply):
        self.reply = reply
        self.calls = []

    def request(self, system, text, images, *, timeout_s, operation):
        self.calls.append((system, text))
        return self.reply


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
              limits=None, clock=None, telemetry=None, execution=None, bare=None):
    return BareVisualGoalNavigation(
        _Source(poses),
        observer or _Observer(),
        planner,
        vehicle or _PathVehicle(),
        geometry or _Geometry(),
        execution or _Execution(),
        limits=limits or VisualGoalLimits(),
        bare=bare or BareNavigationSettings(),
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


def test_the_configured_speed_and_deadline_reach_the_engine():
    """A non-default profile value has to arrive at ``follow_path`` unchanged."""
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=planner, vehicle=vehicle,
        bare=BareNavigationSettings(speed_mps=2.5, path_timeout_s=12.0),
    )
    workflow.run(_stage())
    _, speed_mps, timeout_s = vehicle.calls[0]
    assert (speed_mps, timeout_s) == (2.5, 12.0)


def test_every_waypoint_is_offset_from_the_same_observation_pose():
    """Baseline geometry: five 0.8 m offsets are five points 0.8 m away, not a chain."""
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=planner, vehicle=vehicle,
    )
    workflow.run(_stage())
    points, _, _ = vehicle.calls[0]
    assert [round(point.x, 6) for point in points] == [0.8] * 5
    assert [round(point.yaw_deg, 6) for point in points] == [0.0] * 5


def test_fewer_than_five_waypoints_are_all_executed():
    vehicle = _PathVehicle()
    planner = _Planner((RelativePoseDelta(1.0, 0.0, 0.0, 0.0),) * 3)
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=planner, vehicle=vehicle,
    )
    workflow.run(_stage())
    assert len(vehicle.calls[0][0]) == 3


def test_a_target_bound_on_the_first_frame_is_judged_before_any_planning():
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    workflow = _workflow(
        poses=(WorldPose(9.0, 0.0, 0.0, 0.0),), planner=planner, vehicle=vehicle,
    )
    result = workflow.run(_stage())
    assert result.success is True
    assert planner.calls == 0
    assert vehicle.calls == []


def test_the_batch_is_published_after_the_engine_takes_it():
    events = []
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=_Planner(_seven_points()),
        vehicle=_PathVehicle(events), telemetry=_Telemetry(events),
    )
    workflow.run(_stage())
    assert events[:2] == ["follow", "publish"]


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


def test_a_target_that_is_never_bound_reflys_the_previous_batch():
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    source = _Source((WorldPose(0.0, 0.0, 0.0, 0.0),))
    workflow = BareVisualGoalNavigation(
        source, _Observer(GoalVisibility.NOT_VISIBLE), planner, vehicle,
        _Geometry(), _StopAfter(source, captures=3),
        clock=_Clock(),
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert planner.calls == 1
    assert len(vehicle.calls) == 2
    first, second = vehicle.calls[0][0], vehicle.calls[1][0]
    assert [round(p.x, 6) for p in first] == [round(p.x, 6) for p in second]
    assert "no target was ever bound" in result.summary.reason


def test_a_bound_target_that_is_no_longer_visible_keeps_the_previous_batch():
    """The lost-target trigger is "bound but not seen this round", not "never bound"."""
    planner, vehicle = _Planner(_seven_points()), _PathVehicle()
    source = _Source((WorldPose(0.0, 0.0, 0.0, 0.0),))
    observer = _Observer(statuses=(GoalVisibility.VISIBLE, GoalVisibility.NOT_VISIBLE))
    workflow = BareVisualGoalNavigation(
        source, observer, planner, vehicle,
        _Geometry(), _StopAfter(source, captures=3),
        clock=_Clock(),
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert observer.calls == 2, "detection runs every round, so visibility is observable"
    assert planner.calls == 1, "a round without a visible target must not ask the model"
    assert len(vehicle.calls) == 2
    first, second = vehicle.calls[0][0], vehicle.calls[1][0]
    assert [round(p.x, 6) for p in first] == [round(p.x, 6) for p in second]
    assert "no target was ever bound" not in result.summary.reason


def test_a_visible_bound_target_asks_the_model_every_round():
    observer, planner = _Observer(), _Planner(_seven_points())
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner, observer=observer,
        geometry=_Geometry(world_point=(100.0, 0.0, 0.0)),
        limits=VisualGoalLimits(maximum_planning_rounds=2),
    )
    workflow.run(_stage())
    assert planner.calls == 2
    assert observer.calls == 3


def test_a_repeated_frame_is_rejected_before_any_model_call():
    class _StuckSource(_Source):
        def capture(self, *, budget=None):
            self.captures += 1
            return _observation(WorldPose(0.0, 0.0, 0.0, 0.0), capture_id="same",
                                timestamp_ns=1_000_000)

    planner = _Planner(_seven_points())
    workflow = BareVisualGoalNavigation(
        _StuckSource((WorldPose(0.0, 0.0, 0.0, 0.0),)), _Observer(), planner,
        _PathVehicle(), _Geometry(), _Execution(), clock=_Clock(),
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "perception_unreliable"
    # The first frame is fresh by definition; every repeat of it is rejected
    # before another model request, and two rejections end the run.
    assert planner.calls == 1


def test_prepare_refuses_a_closed_model_caller():
    workflow = _workflow(poses=(WorldPose(0.0, 0.0, 0.0, 0.0),),
                         planner=_Planner(_seven_points()))
    workflow.model_calls.close()
    with pytest.raises(ExecutionError, match="closed or a previous request"):
        workflow.prepare()


def test_the_time_budget_is_checked_after_the_engine_call_returns():
    clock = _Clock()
    vehicle = _FollowThenExpire(clock)
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),),
        planner=_Planner(_seven_points()),
        observer=_Observer(GoalVisibility.NOT_VISIBLE),
        vehicle=vehicle, clock=clock,
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "timeout"
    assert "no target was ever bound" in result.summary.reason
    assert len(vehicle.calls) == 1
    assert result.value.final_distance_m is None


def test_a_parse_failure_ends_the_run_with_that_reason():
    planner = _Planner(())
    planner.error = ProtocolError("trajectory planner returned no JSON array")
    workflow = _workflow(poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner)
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "model_error"
    assert "no JSON array" in result.summary.reason


def test_a_failure_after_binding_does_not_claim_a_missing_target():
    planner = _Planner(())
    planner.error = ProtocolError("trajectory planner returned no JSON array")
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner,
        observer=_Observer(),
    )
    result = workflow.run(_stage())
    assert "no target was ever bound" not in result.summary.reason


def test_a_never_bound_parse_failure_still_says_no_target_was_bound():
    planner = _Planner(())
    planner.error = ProtocolError("trajectory planner returned no JSON array")
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner,
        observer=_Observer(GoalVisibility.NOT_VISIBLE),
    )
    result = workflow.run(_stage())
    assert result.summary.status == "model_error"
    assert "no target was ever bound" in result.summary.reason


def test_a_transient_detection_and_planning_failure_is_retried():
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=_Planner(_seven_points(), failures=1),
        observer=_Observer(failures=1),
    )
    result = workflow.run(_stage())
    assert result.success is True


def test_the_round_budget_counts_model_requests_only():
    planner = _Planner(_seven_points())
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner,
        geometry=_Geometry(world_point=(100.0, 0.0, 0.0)),
        limits=VisualGoalLimits(maximum_planning_rounds=2),
    )
    result = workflow.run(_stage())
    assert result.success is False
    assert result.summary.status == "timeout"
    assert planner.round_indices == [1, 2]


def test_the_round_index_follows_the_actual_request_count():
    planner = _Planner(_seven_points())
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner,
        geometry=_Geometry(world_point=(100.0, 0.0, 0.0)),
        limits=VisualGoalLimits(maximum_planning_rounds=1),
    )
    workflow.run(_stage())
    assert planner.round_indices == [1]


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


def test_the_path_length_counts_vertical_chords():
    planner = _Planner((RelativePoseDelta(0.0, 0.0, 1.0, 0.0),
                        RelativePoseDelta(0.0, 0.0, 2.0, 0.0),
                        RelativePoseDelta(0.0, 0.0, 3.0, 0.0)))
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0),), planner=planner,
        geometry=_Geometry(world_point=(100.0, 0.0, 0.0)),
        limits=VisualGoalLimits(maximum_planning_rounds=1),
    )
    result = workflow.run(_stage())
    assert result.value.path_length_m == 3.0


def test_the_workflow_checks_the_stop_signal_around_every_step():
    execution = _Execution()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(9.0, 0.0, 0.0, 0.0)),
        planner=_Planner(_seven_points()), execution=execution,
    )
    workflow.run(_stage())
    assert execution.checkpoints >= 4


def test_the_bare_chain_asks_for_no_heading_and_flies_zero_yaw_points():
    transport = _Transport("[[2.0,0.0,0.0],[2.0,0.0,0.0]]")
    planner = QwenVLPlanner(
        transport,
        prompt_builder=BareTrajectoryPrompt(),
        parser=parse_translation_trajectory,
    )
    vehicle = _PathVehicle()
    workflow = _workflow(
        poses=(WorldPose(0.0, 0.0, 0.0, 37.0), WorldPose(9.0, 0.0, 0.0, 37.0)),
        planner=planner, vehicle=vehicle,
    )
    result = workflow.run(_stage())
    assert result.success is True
    points, _, _ = vehicle.calls[0]
    assert len(points) == 2
    assert [round(point.yaw_deg, 6) for point in points] == [37.0, 37.0]
    assert "dyaw" not in transport.calls[0][0].lower()
    assert "dyaw" not in transport.calls[0][1].lower()


def test_a_vehicle_without_path_following_is_rejected_before_the_run():
    workflow = _workflow(poses=(WorldPose(0.0, 0.0, 0.0, 0.0),),
                         planner=_Planner(_seven_points()), vehicle=object())
    try:
        workflow.prepare()
    except Exception as exc:
        assert "path-following vehicle" in str(exc)
    else:
        raise AssertionError("a vehicle without follow_path must be rejected")
