from dataclasses import dataclass, field
from typing import Callable
import threading
import math
import logging

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
from planner.extensions.registry import ExtensionRegistry, TaskRegistration
from planner.adapters.task_parser.tasks import builtin_task_codecs
from planner.capabilities.task_validation.basic_motion import BasicMotionStageValidator
from planner.capabilities.task_validation.visual_goal_navigation import VisualGoalStageValidator
from planner.services.execution import ExecutionService
from planner.services.motion import SynchronizedMotionPlanner
from planner.services.trajectory_validation import TrajectoryValidator
from planner.workflows import BasicMotionWorkflow
from planner.workflows.configured import ConfiguredWorkflow
from planner.services.resource_cleanup import close_resources


@dataclass
class Runtime:
    application: PlannerApplication
    connection: AirSimConnection
    close_callbacks: tuple[Callable[[], None], ...] = ()

    _closed: bool = field(default=False, init=False, repr=False)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        close_resources((*self.close_callbacks, self.connection.shutdown))


@dataclass(frozen=True)
class RecordingRuntime:
    recorder: CameraApiRecorder
    connection: AirSimConnection

    def close(self, timeout_s=None):
        return self.recorder.close(self.connection.close_transport, timeout_s)

#“组装和注册不同模块”的入口,注册所有模块，后续根据不同的任务类型走不同的workflow
def build_runtime(config: PlannerConfig) -> Runtime:
    connection = AirSimConnection(config.airsim.host, config.airsim.port, config.airsim.connect_timeout_s)
    vehicle = AirSimVehicle(connection, config.airsim.speed_mps, config.airsim.move_timeout_s,
                           config.motion.limits)
    motion_planner = SynchronizedMotionPlanner(config.airsim.speed_mps, config.motion.limits)
    execution = ExecutionService(vehicle)
    observation_gate = ObservationGate(vehicle, config.motion.limits, config.airsim.move_timeout_s,
        owner_thread_id=threading.get_ident(), check_safety=execution.check_safety)
    execution.bind_observation_checkpoint(observation_gate.service_requests, observation_gate.cancel_requests)
    vehicle.motion_tracker.observation_gate = observation_gate
    observation = AirSimObservationSource(
        connection, vehicle, config.airsim.camera_id, config.depth.min_m, config.depth.max_m,
        observation_gate=observation_gate,
        depth_horizontal_fov_deg=config.airsim.depth_horizontal_fov_deg,
        verify_image_pair=True,
        maximum_pair_time_gap_s=config.airsim.maximum_pair_time_gap_s,
        depth_mapping=config.depth.mapping,
    )
    validator = TrajectoryValidator(config.trajectory.point_count, config.trajectory.max_step_m,
                                    config.trajectory.max_yaw_step_deg)
    workflow = BasicMotionWorkflow(
        observation, BasicMotionConverter(), BasicMotionSafety(config.basic_motion.free_space),
        BasicMotionCompletion(config.motion.limits), validator, motion_planner, execution,
        config.motion.limits, config.airsim.move_timeout_s,
        collision_check=vehicle.collision_state if config.basic_motion.collision_check_enabled else None,
    )
    registry = ExtensionRegistry()
    codecs = {codec.kind: codec for codec in builtin_task_codecs()}
    registry.register(TaskRegistration(TaskKind.BASIC_MOTION, codecs[TaskKind.BASIC_MOTION],
        BasicMotionStageValidator(config.trajectory.max_step_m, config.trajectory.max_yaw_step_deg,
                                  config.basic_motion.free_space.confirmed), workflow))
    if config.visual_goal.enabled:
        from planner.visual_goal_bootstrap import build_visual_goal_workflow
        visual_workflow = ConfiguredWorkflow(lambda: build_visual_goal_workflow(
            config, observation, motion_planner, execution, validator))
        registry.register(TaskRegistration(TaskKind.VISUAL_GOAL_NAVIGATION,
            codecs[TaskKind.VISUAL_GOAL_NAVIGATION],
            VisualGoalStageValidator(config.visual_goal.limits, config.depth.max_m, config.visual_goal.completion_rules),
            visual_workflow))
    tasks = registry.tasks
    router = WorkflowRouter({task.kind: task.workflow for task in tasks})
    parser = OpenAITaskParser(
        config.task_parser.url, config.task_parser.model,
        config.task_parser.api_key, config.task_parser.timeout_s,
        supported_kinds=router.supported_kinds, codecs=tuple(task.codec for task in tasks),
        max_completion_tokens=config.task_parser.max_completion_tokens,
        reasoning_effort=config.task_parser.reasoning_effort,
        request_format=config.task_parser.request_format,
    )
    app = PlannerApplication(parser, MissionValidator({task.kind: task.validator for task in tasks}),
                             MissionRunner(router))
    return Runtime(app, connection, tuple(task.workflow.close for task in reversed(tasks)))


def build_recording_runtime(
    config: PlannerConfig,
    *,
    output_root: str | None = None,
    fps: float | None = None,
) -> RecordingRuntime:
    """Build an independent read-only Camera API capture connection."""
    capture_fps = config.recording.fps if fps is None else fps
    if isinstance(capture_fps, bool) or not isinstance(capture_fps, (int, float)) or not math.isfinite(capture_fps) or capture_fps <= 0:
        raise ValueError("recording fps must be finite and positive")
    connection = AirSimConnection(config.airsim.host, config.airsim.port, config.airsim.connect_timeout_s)
    try:
        vehicle = AirSimVehicle(connection, config.airsim.speed_mps, config.airsim.move_timeout_s)
        source = AirSimObservationSource(
            connection, vehicle, config.airsim.camera_id, config.depth.min_m, config.depth.max_m,
            depth_mapping=getattr(config.depth, "mapping", "log_inverse_gray_v1"),
        )
        recorder = CameraApiRecorder(
            source,
            output_root or config.recording.output_root,
            capture_fps,
        )
        return RecordingRuntime(recorder, connection)
    except BaseException:
        try:
            connection.close_transport()
        except Exception:
            logging.getLogger(__name__).exception("Failed to close recording transport after assembly failure")
        raise
