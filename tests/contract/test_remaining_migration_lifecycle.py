from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import threading
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

from planner.adapters.airsim.observation_gate import ObservationGate
from planner.domain.motion import MotionLimits
from planner.domain.mission import MissionStage, TaskKind
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.trajectory import MotionTrajectory, MotionSegment
from planner.domain.pose import WorldPose
from planner.domain.visual_goal import VisualGoalStatus
from planner.errors import ExecutionError, ConfigurationError
from planner.services.execution import ExecutionService
from planner.workflows.basic_motion import BasicMotionWorkflow
from tests.fakes.components import FakeVehicle
from tests.fakes.visual_goal import Clock, goal_observation
from tests.integration.test_visual_goal_workflow import setup


def test_model_timeout_quarantines_goal_result_and_blocks_basic_stage():
    workflow, stage, source, observer, planner, vehicle, clock = setup()
    release, entered = threading.Event(), threading.Event()
    workflow.limits = replace(workflow.limits, model_request_timeout_s=.05)
    def delayed(*args, **kwargs):
        entered.set()
        clock.sleep(.1)
        assert release.wait(2)
        return goal_observation()
    observer.observe = delayed
    try:
        result = workflow.run(stage)
        assert entered.is_set() and result.value.status is VisualGoalStatus.TIMEOUT
        assert workflow.model_calls.has_pending_call
        basic = BasicMotionWorkflow(vehicle, workflow.execution, clock)
        next_stage = MissionStage("basic", TaskKind.BASIC_MOTION,
            BasicMotionParameters("turn", 0, 0, 0, 30))
        blocked = basic.run(next_stage)
        assert not blocked.success and "previous request" in blocked.error
        assert not vehicle.segments and not planner.calls
    finally:
        release.set()
        workflow.close()


def test_formal_capture_is_serviced_by_control_owner():
    clock, vehicle = Clock(), FakeVehicle()
    execution = ExecutionService(vehicle, clock)
    owner = threading.get_ident()
    gate = ObservationGate(vehicle, MotionLimits(), 5, clock, clock.sleep,
        owner_thread_id=owner, check_safety=execution.check_deadline)
    execution.bind_observation_checkpoint(gate.service_requests, gate.cancel_requests)
    def direct_capture():
        with gate.capture():
            return 1
    with ThreadPoolExecutor(1) as pool:
        with pytest.raises(ExecutionError, match="owner"):
            pool.submit(direct_capture).result()
        future = pool.submit(gate.request_capture, threading.get_ident).result()
    assert not future.done()
    execution.checkpoint()
    assert future.result() == owner and vehicle.hovered


def test_deadline_after_segment_keeps_measured_receipt_and_only_tail():
    clock, vehicle = Clock(), FakeVehicle()
    execution = ExecutionService(vehicle, clock)
    poses = [WorldPose(x, 0, 0, 0) for x in range(3)]
    trajectory = MotionTrajectory(tuple(MotionSegment(a, b, "synchronized_quintic") for a,b in zip(poses, poses[1:])))
    original = vehicle.execute_segment
    def run(segment, **kwargs):
        original(segment, **kwargs)
        clock.sleep(2)
    vehicle.execute_segment = run
    execution.set_deadline(clock()+1)
    with pytest.raises(ExecutionError) as failure:
        execution.execute_with_receipt(trajectory)
    receipt = failure.value.receipt
    assert receipt.completed_segments == 1 and len(receipt.measured_actions) == 1
    assert receipt.measured_actions[0].dx == 1
    assert len(receipt.remaining.segments) == 1 and receipt.remaining.segments[0].end.x == 2


def test_recording_invalid_fps_is_rejected_before_connection(monkeypatch):
    from planner import bootstrap
    connection = Mock()
    monkeypatch.setattr(bootstrap, "AirSimConnection", connection)
    with pytest.raises(ValueError):
        bootstrap.build_recording_runtime(SimpleNamespace(recording=SimpleNamespace(fps=5)), fps=float("nan"))
    connection.assert_not_called()


def test_progress_review_reservation_is_explicit(tmp_path):
    from config import load_config
    from tests.fakes.configuration import main_config_data, write_config
    raw = main_config_data()
    raw["progress_review"]["enabled"] = True
    path = write_config(tmp_path / "review.yaml", raw)
    with pytest.raises(ConfigurationError, match="reserved"):
        load_config(path)
