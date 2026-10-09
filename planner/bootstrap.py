from dataclasses import dataclass

from config.schema import PlannerConfig
from planner.adapters.airsim import (
    AirSimConnection,
    AirSimObservationSource,
    AirSimVehicle,
    CameraApiRecorder,
)
from planner.adapters.airsim.observation_gate import ObservationGate
from planner.adapters.task_parser import OpenAITaskParser
from planner.capabilities.basic_motion_conversion import BasicMotionConverter
from planner.capabilities.basic_motion_safety import BasicMotionSafety
from planner.capabilities.completion.basic_motion import BasicMotionCompletion
from planner.application import PlannerApplication
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import TaskKind
from planner.services.execution import ExecutionService
from planner.services.motion import SynchronizedMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows import BasicMotionWorkflow


@dataclass(frozen=True)
class Runtime:
    application: PlannerApplication
    connection: AirSimConnection


@dataclass(frozen=True)
class RecordingRuntime:
    recorder: CameraApiRecorder
    connection: AirSimConnection

#“组装和注册不同模块”的入口,注册所有模块，后续根据不同的任务类型走不同的workflow
def build_runtime(config: PlannerConfig) -> Runtime:
    connection = AirSimConnection(config.airsim.host, config.airsim.port, config.airsim.connect_timeout_s)
    vehicle = AirSimVehicle(connection, config.airsim.speed_mps, config.airsim.move_timeout_s,
                           config.motion.limits)
    motion_planner = SynchronizedMotionPlanner(config.airsim.speed_mps, config.motion.limits)
    observation_gate = ObservationGate(vehicle, config.motion.limits, config.airsim.move_timeout_s)
    vehicle.motion_tracker.observation_gate = observation_gate
    observation = AirSimObservationSource(
        connection, vehicle, config.airsim.camera_id, config.depth.min_m, config.depth.max_m,
        observation_gate=observation_gate,
        depth_horizontal_fov_deg=(config.visual_goal.depth_horizontal_fov_deg if config.visual_goal.enabled else None),
        verify_image_pair=config.visual_goal.enabled,
    )
    execution = ExecutionService(vehicle)
    validator = TrajectoryValidator(config.trajectory.point_count, config.trajectory.max_step_m,
                                    config.trajectory.max_yaw_step_deg)
    workflow = BasicMotionWorkflow(
        observation, BasicMotionConverter(), BasicMotionSafety(config.basic_motion.free_space),
        BasicMotionCompletion(config.motion.limits), validator, motion_planner, execution,
        config.motion.limits, config.airsim.move_timeout_s,
    )
    workflows = {TaskKind.BASIC_MOTION: workflow}
    if config.visual_goal.enabled:
        from planner.visual_goal_bootstrap import build_visual_goal_workflow
        workflows[TaskKind.VISUAL_GOAL_NAVIGATION] = build_visual_goal_workflow(
            config, observation, motion_planner, execution, validator,
        )
    router = WorkflowRouter(workflows)
    parser = OpenAITaskParser(
        config.task_parser.url, config.task_parser.model,
        config.task_parser.api_key, config.task_parser.timeout_s,
        supported_kinds=router.supported_kinds,
    )
    app = PlannerApplication(parser, MissionValidator(router.supported_kinds,
        config.visual_goal.limits if config.visual_goal.enabled else None, config.depth.max_m,
        maximum_step_m=config.trajectory.max_step_m,
        maximum_yaw_step_deg=config.trajectory.max_yaw_step_deg,
        basic_motion_ready=config.basic_motion.free_space.confirmed,
    ), MissionRunner(router))
    return Runtime(app, connection)


def build_recording_runtime(
    config: PlannerConfig,
    *,
    output_root: str | None = None,
    fps: float | None = None,
) -> RecordingRuntime:
    """Build an independent read-only Camera API capture connection."""
    connection = AirSimConnection(config.airsim.host, config.airsim.port, config.airsim.connect_timeout_s)
    vehicle = AirSimVehicle(connection, config.airsim.speed_mps, config.airsim.move_timeout_s)
    source = AirSimObservationSource(
        connection, vehicle, config.airsim.camera_id, config.depth.min_m, config.depth.max_m,
    )
    recorder = CameraApiRecorder(
        source,
        output_root or config.recording.output_root,
        config.recording.fps if fps is None else fps,
    )
    return RecordingRuntime(recorder, connection)
