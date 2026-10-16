"""Direct-action integration: no AirSim connection or model request."""
import pytest
from planner.application.application import PlannerApplication
from planner.application.mission_runner import MissionRunner
from tests.fakes.missions import mission_validator
from planner.application.workflow_router import WorkflowRouter
from planner.domain.basic_motion import BasicMotionParameters, BasicMotionStatus
from planner.domain.mission import MissionPlan, MissionStage, TaskKind
from planner.domain.pose import WorldPose
from planner.domain.result import CompletionBasis
from planner.services.execution import ExecutionService
from planner.workflows.basic_motion import BasicMotionWorkflow
from tests.fakes.visual_goal import Clock


class Vehicle:
    def __init__(self):
        self.pose=WorldPose(0,0,-4,0)
        self.moves=[]
        self.failure=None
        self.stalled=False
        self.reads=0
        self.fail_final_read=False
        self.cancelled=self.hovered=0

    def current_pose(self):
        self.reads+=1
        if self.fail_final_read and self.reads>1: raise RuntimeError("pose unavailable")
        return self.pose

    def execute_basic_motion(self,start,target,yaw_delta_deg):
        self.moves.append((start,target,yaw_delta_deg))
        if self.failure: raise self.failure
        if not self.stalled: self.pose=target

    def collision_state(self):
        raise AssertionError("basic motion must not read collision")

    def cancel(self): self.cancelled+=1
    def hover(self): self.hovered+=1


def build():
    clock=Clock(); vehicle=Vehicle()
    execution=ExecutionService(vehicle,clock)
    # Formal observations/checked trajectory execution must never be used here.
    execution.bind_observation_checkpoint(lambda: pytest.fail("observation requested"),lambda: None)
    execution.execute_with_receipt=lambda *a,**k: pytest.fail("checked execution requested")
    return BasicMotionWorkflow(vehicle,execution,clock),vehicle


def stage(**fields):
    return MissionStage("s1",TaskKind.BASIC_MOTION,BasicMotionParameters("explicit motion",**fields))


@pytest.mark.parametrize("fields",[{"up_m":10},{"forward_m":100},{"yaw_right_deg":450},
                                  {"forward_m":8,"right_m":5,"yaw_right_deg":15}])
def test_action_is_sent_once_without_flight_admission_or_observation(fields):
    workflow,vehicle=build()
    result=workflow.run(stage(**fields))
    assert result.success and len(vehicle.moves)==1
    assert result.summary.completion_basis is CompletionBasis.API_EXECUTION
    assert not result.value.stop.confirmed
    assert result.value.completion_basis=="api_execution"


def test_altitude_is_relative_to_current_takeoff_position():
    workflow,vehicle=build();vehicle.pose=WorldPose(12,3,-1.675,0)
    result=workflow.run(stage(up_m=10))
    assert result.value.target==WorldPose(12,3,-11.675,0)


def test_sequential_commands_use_new_actual_heading():
    workflow,vehicle=build()
    assert workflow.run(stage(yaw_right_deg=90)).success
    assert workflow.run(stage(forward_m=5)).success
    assert vehicle.pose.x==pytest.approx(0,abs=1e-6) and vehicle.pose.y==pytest.approx(5)


@pytest.mark.parametrize("stalled,missing_pose",[(True,False),(False,True)])
def test_no_postflight_verification_blocks_api_completion(stalled,missing_pose):
    workflow,vehicle=build();vehicle.stalled=stalled;vehicle.fail_final_read=missing_pose
    assert workflow.run(stage(forward_m=2)).success
    assert len(vehicle.moves)==1


def test_api_exception_stops_and_is_not_replayed():
    workflow,vehicle=build();vehicle.failure=RuntimeError("RPC unavailable")
    result=workflow.run(stage(up_m=10))
    assert not result.success and result.value.status is BasicMotionStatus.EXECUTION_ERROR
    assert len(vehicle.moves)==1 and vehicle.cancelled and vehicle.hovered


def test_keyboard_interrupt_stops_and_propagates():
    workflow,vehicle=build();vehicle.failure=KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt): workflow.run(stage(up_m=10))
    assert vehicle.cancelled and vehicle.hovered


def test_mission_runs_large_basic_actions_without_trajectory_limits():
    workflow,vehicle=build()
    plan=MissionPlan("mission_plan_v1",(stage(up_m=10),
        MissionStage("s2",TaskKind.BASIC_MOTION,BasicMotionParameters("forward",forward_m=100))))
    class Parser:
        def parse(self,instruction): return plan
    app=PlannerApplication(Parser(),mission_validator({TaskKind.BASIC_MOTION}),
        MissionRunner(WorkflowRouter({TaskKind.BASIC_MOTION:workflow})))
    result=app.execute("up then forward")
    assert result.success and len(result.stages)==2 and len(vehicle.moves)==2
