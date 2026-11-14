import math

import pytest

from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.domain.basic_motion import (
    BasicMotionParameters,
    BasicMotionStatus,
)
from planner.domain.execution import StopResult
from planner.domain.mission import MissionStage, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.domain.result import CompletionBasis
from planner.workflows.basic_motion import BasicMotionWorkflow


def test_parameters_require_a_finite_non_zero_action():
    with pytest.raises(ValueError):
        BasicMotionParameters("turn", forward_m=0.0, yaw_right_deg=0.0)
    with pytest.raises(ValueError):
        BasicMotionParameters("turn", forward_m=True)
    with pytest.raises(ValueError):
        BasicMotionParameters("turn", yaw_right_deg=False)
    with pytest.raises(ValueError):
        BasicMotionParameters("turn", forward_m=float("nan"))
    with pytest.raises(ValueError):
        BasicMotionParameters("turn", yaw_right_deg=float("inf"))
    with pytest.raises(ValueError):
        BasicMotionParameters("turn", forward_m="5")
    with pytest.raises(ValueError):
        BasicMotionParameters("   ", forward_m=5.0)


def test_parameters_allow_reverse_left_and_simultaneous_actions():
    reverse = BasicMotionParameters("reverse 2m", forward_m=-2.0)
    left = BasicMotionParameters("turn left", yaw_right_deg=-90.0)
    combined = BasicMotionParameters(
        "forward and right", forward_m=5.0, yaw_right_deg=45.0
    )
    assert reverse.forward_m == -2.0
    assert left.yaw_right_deg == -90.0
    assert combined.forward_m == 5.0
    assert combined.yaw_right_deg == 45.0


def _start(yaw_deg=0.0):
    return WorldPose(0.0, 0.0, 0.0, yaw_deg)


@pytest.mark.parametrize(
    ("parameters", "start", "expected"),
    [
        (
            BasicMotionParameters("forward 5m", forward_m=5.0),
            _start(),
            WorldPose(5.0, 0.0, 0.0, 0.0),
        ),
        (
            BasicMotionParameters("reverse 2m", forward_m=-2.0),
            _start(),
            WorldPose(-2.0, 0.0, 0.0, 0.0),
        ),
        (
            BasicMotionParameters("turn left", yaw_right_deg=-90.0),
            _start(),
            WorldPose(0.0, 0.0, 0.0, -90.0),
        ),
        (
            BasicMotionParameters("turn right", yaw_right_deg=90.0),
            _start(),
            WorldPose(0.0, 0.0, 0.0, 90.0),
        ),
        (
            BasicMotionParameters(
                "forward 5m and right 45deg",
                forward_m=5.0,
                yaw_right_deg=45.0,
            ),
            _start(),
            WorldPose(5.0, 0.0, 0.0, 45.0),
        ),
        (
            BasicMotionParameters("forward 5m", forward_m=5.0),
            _start(90.0),
            WorldPose(0.0, 5.0, 0.0, 90.0),
        ),
        (
            BasicMotionParameters("turn right", yaw_right_deg=20.0),
            _start(350.0),
            WorldPose(0.0, 0.0, 0.0, 10.0),
        ),
    ],
)
def test_converter_uses_the_ground_heading(parameters, start, expected):
    target = BasicMotionConverter().create_target(parameters, start)
    assert target.x == pytest.approx(expected.x)
    assert target.y == pytest.approx(expected.y)
    assert target.z == start.z
    assert target.yaw_deg == pytest.approx(expected.yaw_deg)


class FakeVehicle:
    def __init__(self, pose):
        self.pose = pose

    def current_pose(self):
        return self.pose


class FakeMotion:
    def __init__(self):
        self.speed_mps = 3.0
        self.limits = MotionLimits()


class FakeExecution:
    def __init__(self, vehicle, *, reach=True, interrupt=False):
        self.vehicle = vehicle
        self.reach = reach
        self.interrupt = interrupt
        self.acquired = []
        self.released = []
        self.stops = 0
        self.executed = []

    def acquire_flight(self, owner):
        self.acquired.append(owner)

    def release_flight(self, owner):
        self.released.append(owner)

    def stop_if_owner(self, owner):
        self.stops += 1
        return StopResult(False, ("cancel: unavailable",))

    def execute(self, trajectory):
        self.executed.append(trajectory)
        if self.interrupt:
            raise KeyboardInterrupt
        if self.reach:
            self.vehicle.pose = trajectory.segments[-1].end


def _workflow(parameters, *, reach=True, interrupt=False):
    vehicle = FakeVehicle(_start())
    execution = FakeExecution(
        vehicle, reach=reach, interrupt=interrupt
    )
    workflow = BasicMotionWorkflow(vehicle, FakeMotion(), execution)
    stage = MissionStage("stage_1", TaskKind.BASIC_MOTION, parameters)
    return workflow, execution, vehicle, stage


def test_workflow_completes_and_confirms_by_local_measurement():
    workflow, execution, vehicle, stage = _workflow(
        BasicMotionParameters("forward 5m", forward_m=5.0)
    )
    result = workflow.run(stage)
    assert result.success is True
    assert result.stage_id == "stage_1"
    assert result.value.status is BasicMotionStatus.COMPLETED
    assert result.summary.completion_basis is CompletionBasis.LOCAL_MEASUREMENT
    assert result.summary.completion_confirmed is True
    assert result.summary.final_pose == vehicle.pose
    assert vehicle.pose.x == pytest.approx(5.0)
    assert execution.acquired == [workflow]
    assert execution.released == [workflow]
    assert execution.stops == 0


def test_workflow_reports_a_pose_tolerance_failure():
    workflow, execution, _, stage = _workflow(
        BasicMotionParameters("forward 5m", forward_m=5.0), reach=False
    )
    result = workflow.run(stage)
    assert result.success is False
    assert "position/yaw" in result.error
    assert result.summary.completion_basis is CompletionBasis.NOT_VERIFIED
    assert result.summary.completion_confirmed is False
    assert result.value.status is BasicMotionStatus.EXECUTION_ERROR
    assert result.value.stop.confirmed is False
    assert execution.stops == 1
    assert execution.released == [workflow]


def test_workflow_propagates_cancellation_after_stopping():
    workflow, execution, _, stage = _workflow(
        BasicMotionParameters("forward 5m", forward_m=5.0), interrupt=True
    )
    with pytest.raises(KeyboardInterrupt):
        workflow.run(stage)
    assert execution.stops == 1
    assert execution.released == [workflow]


def test_workflow_executes_full_turns_without_the_180_degree_cap():
    workflow, execution, vehicle, stage = _workflow(
        BasicMotionParameters("turn right 450deg", yaw_right_deg=450.0)
    )
    result = workflow.run(stage)
    assert result.success is True
    segment = execution.executed[0].segments[0]
    assert segment.yaw_delta_deg == 450.0
    assert segment.end.yaw_deg == pytest.approx(90.0)
    assert segment.duration_s >= 1.875 * 450.0 / 30.0
    assert vehicle.pose.yaw_deg == pytest.approx(90.0)