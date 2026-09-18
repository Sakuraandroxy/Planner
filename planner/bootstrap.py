from dataclasses import dataclass

from config.schema import PlannerConfig
from planner.adapters.airsim import AirSimConnection, AirSimObservationSource, AirSimVehicle
from planner.adapters.task_parser import OpenAITaskParser
from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.application import PlannerApplication
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import TaskKind
from planner.services.execution import ExecutionService
from planner.services.motion import PassThroughMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows import NavigationWorkflow


@dataclass(frozen=True)
class Runtime:
    application: PlannerApplication
    connection: AirSimConnection


def build_runtime(config: PlannerConfig) -> Runtime:
    connection = AirSimConnection(config.airsim.host, config.airsim.port, config.airsim.connect_timeout_s)
    vehicle = AirSimVehicle(connection, config.airsim.speed_mps, config.airsim.move_timeout_s)
    observation = AirSimObservationSource(
        connection, vehicle, config.airsim.camera_id, config.depth.min_m, config.depth.max_m
    )
    planner = QwenVLPlanner(
        config.trajectory_planner.url, config.trajectory_planner.model,
        config.trajectory_planner.api_key, config.trajectory_planner.timeout_s,
        config.trajectory.point_count,
    )
    workflow = NavigationWorkflow(
        observation, planner,
        TrajectoryValidator(
            config.trajectory.point_count, config.trajectory.max_step_m,
            config.trajectory.max_yaw_step_deg,
        ),
        PassThroughMotionPlanner(config.airsim.speed_mps), ExecutionService(vehicle),
    )
    router = WorkflowRouter({TaskKind.NAVIGATION: workflow})
    parser = OpenAITaskParser(
        config.task_parser.url, config.task_parser.model,
        config.task_parser.api_key, config.task_parser.timeout_s,
    )
    app = PlannerApplication(parser, MissionValidator(router.supported_kinds), MissionRunner(router))
    return Runtime(app, connection)

