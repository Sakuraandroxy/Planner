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
from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.prompts import NavigationProgressPrompt, NavigationTrajectoryPrompt
from planner.application import PlannerApplication
from planner.application.mission_runner import MissionRunner
from planner.application.mission_validator import MissionValidator
from planner.application.workflow_router import WorkflowRouter
from planner.domain.mission import TaskKind
from planner.services.execution import ExecutionService
from planner.services.motion import SynchronizedMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows import NavigationWorkflow
from planner.workflows.termination import NavigationExitPolicy
from planner.adapters.trajectory_planner.progress_review import VisualProgressReviewer


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
    trajectory_prompt = NavigationTrajectoryPrompt()
    planner = QwenVLPlanner(
        config.trajectory_planner.url, config.trajectory_planner.model,
        config.trajectory_planner.api_key, config.trajectory_planner.timeout_s,
        config.trajectory.point_count,
        thinking=config.trajectory_planner.thinking,
        prompt_builder=trajectory_prompt,
    )
    workflow = NavigationWorkflow(
        observation, planner,
        TrajectoryValidator(
            config.trajectory.point_count, config.trajectory.max_step_m,
            config.trajectory.max_yaw_step_deg,
        ),
        motion_planner, ExecutionService(vehicle),
        reviewer=(VisualProgressReviewer(planner, NavigationProgressPrompt())
                  if config.navigation_max_rounds > 1 else None),
        exit_policy=NavigationExitPolicy(config.navigation_max_rounds),
    )
    workflows = {TaskKind.NAVIGATION: workflow}
    if config.visual_goal.enabled:
        from planner.visual_goal_bootstrap import build_visual_goal_workflow
        workflows[TaskKind.VISUAL_GOAL_NAVIGATION] = build_visual_goal_workflow(
            config, observation, motion_planner, ExecutionService(vehicle),
            TrajectoryValidator(config.trajectory.point_count, config.trajectory.max_step_m,
                                config.trajectory.max_yaw_step_deg),
        )
    router = WorkflowRouter(workflows)
    parser = OpenAITaskParser(
        config.task_parser.url, config.task_parser.model,
        config.task_parser.api_key, config.task_parser.timeout_s,
        supported_kinds=router.supported_kinds,
    )
    app = PlannerApplication(parser, MissionValidator(router.supported_kinds,
        config.visual_goal.limits if config.visual_goal.enabled else None, config.depth.max_m), MissionRunner(router))
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
