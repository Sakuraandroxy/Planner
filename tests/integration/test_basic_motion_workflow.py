"""Offline assembly with measured-pose fakes: never connects or takes off."""
from dataclasses import replace

import pytest

from planner.application.application import PlannerApplication
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.capabilities.basic_motion_safety import BasicMotionSafety
from planner.capabilities.completion.basic_motion import BasicMotionCompletion
from planner.domain.basic_motion import BasicMotionParameters, BasicMotionStatus, ConfirmedFreeSpace
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.errors import ExecutionError, ObservationError, ProtocolError
from planner.services.execution import ExecutionService
from planner.services.motion.synchronized import SynchronizedMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows.basic_motion import BasicMotionWorkflow
from tests.fakes.visual_goal import Clock, wall_observation


class Vehicle:
    def __init__(self, clock):
        self.clock, self.pose = clock, WorldPose(0, 0, -4, 0)
        self.moves = []
        self.stalled = self.collided = False
        self.failure = None
        self.cancelled = self.hovered = 0

    def current_pose(self):
        return self.pose

    def execute_segment(self, segment, *, budget=None):
        self.moves.append(segment)
        self.clock.sleep(segment.duration_s)
        if self.failure:
            raise self.failure
        if not self.stalled:
            self.pose = segment.end

    def collision_state(self):
        return self.collided

    def cancel(self):
        self.cancelled += 1

    def hover(self):
        self.hovered += 1


class Source:
    def __init__(self, vehicle, clock):
        self.vehicle, self.clock = vehicle, clock
        self.calls, self.failure_at, self.stopped, self.delay = 0, None, True, 0.01

    def capture(self, *, budget=None):
        self.calls += 1
        if self.calls == self.failure_at:
            raise ObservationError("capture unavailable")
        self.clock.sleep(self.delay)
        return replace(wall_observation(pose=self.vehicle.pose, clock=self.clock, stamp=self.calls),
                       stopped=self.stopped)


def build(confirmed=True, timeout=60):
    clock = Clock()
    vehicle = Vehicle(clock)
    source = Source(vehicle, clock)
    limits = MotionLimits()
    workflow = BasicMotionWorkflow(source, BasicMotionConverter(),
        BasicMotionSafety(ConfirmedFreeSpace(confirmed, (-20, -20, -30), (20, 20, 5), 1)),
        BasicMotionCompletion(limits), TrajectoryValidator(5, 30, 180),
        SynchronizedMotionPlanner(2, limits), ExecutionService(vehicle, clock), limits, timeout, clock)
    return workflow, vehicle, source


def stage(**fields):
    return MissionStage("s1", TaskKind.BASIC_MOTION, BasicMotionParameters("explicit motion", **fields))


def test_up12_is_one_measured_action_not_visual_goal_height_limited():
    workflow, vehicle, source = build()
    result = workflow.run(stage(up_m=12))
    assert result.success and result.value.status is BasicMotionStatus.COMPLETED
    assert vehicle.pose.z == -16
    assert len(vehicle.moves) == 1 and source.calls == 2
    assert result.value.completion_basis == "measured_pose_and_stopped_observation"
    assert result.value.stop.confirmed


def test_simultaneous_translation_and_rotation_is_one_motion():
    workflow, vehicle, source = build()
    assert workflow.run(stage(forward_m=8, right_m=5, yaw_right_deg=15)).success
    assert vehicle.pose == WorldPose(8, 5, -4, 15)
    assert len(vehicle.moves) == 1


def test_repeated_stages_use_actual_heading_and_reset_relative_origin():
    workflow, vehicle, source = build()
    assert workflow.run(stage(yaw_right_deg=90)).success
    assert workflow.run(stage(forward_m=5)).success
    assert vehicle.pose.x == pytest.approx(0, abs=1e-6)
    assert vehicle.pose.y == pytest.approx(5)
    assert len(vehicle.moves) == 2 and source.calls == 4


@pytest.mark.parametrize("confirmed,outside", [(False, False), (True, True)])
def test_unconfirmed_or_out_of_box_never_flies(confirmed, outside):
    workflow, vehicle, source = build(confirmed)
    if outside:
        vehicle.pose = WorldPose(19.5, 0, -4, 0)
    result = workflow.run(stage(yaw_right_deg=90))
    assert not result.success and result.value.status is BasicMotionStatus.BLOCKED
    assert not vehicle.moves
    assert vehicle.cancelled and vehicle.hovered


def test_final_api_success_does_not_hide_stalled_vehicle():
    workflow, vehicle, source = build()
    vehicle.stalled = True
    result = workflow.run(stage(forward_m=5))
    assert not result.success and result.value.status is BasicMotionStatus.UNCONFIRMED
    assert len(vehicle.moves) == 1  # Never repeat the original relative command.
    assert vehicle.cancelled and vehicle.hovered


@pytest.mark.parametrize("failure_at", [1, 2])
def test_capture_failure_stops_and_never_replays(failure_at):
    workflow, vehicle, source = build()
    source.failure_at = failure_at
    result = workflow.run(stage(forward_m=2))
    assert not result.success and result.value.status is BasicMotionStatus.OBSERVATION_ERROR
    assert len(vehicle.moves) == failure_at-1
    assert vehicle.cancelled and vehicle.hovered
    if failure_at == 2:
        assert result.value.actual == vehicle.pose


def test_stopped_evidence_is_required_before_motion():
    workflow, vehicle, source = build()
    source.stopped = False
    assert not workflow.run(stage(forward_m=2)).success
    assert not vehicle.moves


def test_late_capture_is_not_executed():
    workflow, vehicle, source = build(timeout=1)
    source.delay = 2
    result = workflow.run(stage(forward_m=2))
    assert result.value.status is BasicMotionStatus.TIMEOUT
    assert not vehicle.moves


def test_collision_or_execution_error_is_failure_and_stops():
    for collision in (True, False):
        workflow, vehicle, source = build()
        vehicle.collided = collision
        vehicle.failure = None if collision else ExecutionError("flight failed")
        result = workflow.run(stage(forward_m=2))
        assert not result.success and result.value.status is BasicMotionStatus.EXECUTION_ERROR
        assert vehicle.cancelled and vehicle.hovered


def test_keyboard_interrupt_propagates_after_stop():
    workflow, vehicle, source = build()
    vehicle.failure = KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt):
        workflow.run(stage(forward_m=2))
    assert vehicle.cancelled and vehicle.hovered


def test_complete_plan_is_validated_before_first_stage_flies():
    workflow, vehicle, source = build()
    plan = MissionPlan("mission_plan_v1", (stage(up_m=2),
        MissionStage("s2", TaskKind.BASIC_MOTION, BasicMotionParameters("too far", forward_m=100))))
    class Parser:
        def parse(self, instruction):
            return plan
    runner = MissionRunner(WorkflowRouter({TaskKind.BASIC_MOTION: workflow}))
    app = PlannerApplication(Parser(), MissionValidator({TaskKind.BASIC_MOTION}), runner)
    with pytest.raises(ProtocolError):
        app.execute("long mission")
    assert not vehicle.moves and source.calls == 0
