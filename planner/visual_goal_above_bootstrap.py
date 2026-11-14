"""Explicit above-task assembly; no task flow or model parsing lives here."""
from contextlib import ExitStack

from planner.adapters.airsim import AirSimConnection, AirSimObservationSource
from planner.adapters.airsim.above_camera import AirSimCameraView
from planner.adapters.airsim.above_observation import AboveObservationSource
from planner.adapters.diagnostics.above_trace import AboveTrace
from planner.adapters.diagnostics.background_trace import BackgroundTrace
from planner.adapters.multimodal_chat import MultimodalChatClient
from planner.adapters.target_detection.above_client import OpenAICompatibleTopDetector
from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.prompts.visual_goal_above import VisualGoalAboveTrajectoryPrompt
from planner.adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from planner.adapters.task_completion.prompts.visual_goal_above import VisualGoalAboveCompletionPrompt
from planner.capabilities.targeting.top_geometry import TopGeometry
from planner.capabilities.targeting.above_target_association import AboveTargetAssociation
from planner.capabilities.viewing.temporary_view import TemporaryView
from planner.errors import ConfigurationError
from planner.workflows.visual_goal_above import VisualGoalAboveWorkflow
from planner.workflows.visual_goal_above.flight import AboveFlight
from planner.workflows.visual_goal_above.perception import AbovePerception


def build_visual_goal_above_workflow(config, connection, holding, execution, validator, *, observation_gate,
                                    connection_factory=AirSimConnection):
    detector = config.target_detection
    if detector is None or not detector.api_key.strip() or detector.api_key == "no-key":
        raise ConfigurationError("visual_goal_above requires target_detection and its API key")
    if not config.task_completion.api_key.strip() or config.task_completion.api_key == "no-key":
        raise ConfigurationError("visual_goal_above requires task_completion and its API key")
    settings,backend,completion = config.visual_goal_above,config.trajectory_planner,config.task_completion
    transport = MultimodalChatClient(detector.url,detector.model,detector.api_key,detector.timeout_s,
        max_completion_tokens=detector.max_completion_tokens,reasoning_effort=detector.reasoning_effort,
        request_format=detector.request_format)
    perception = AbovePerception(OpenAICompatibleTopDetector(transport,detector.max_image_side),
        TopGeometry(config.depth.min_m),
        AboveTargetAssociation())
    planner = QwenVLPlanner(backend.url,backend.model,backend.api_key,backend.timeout_s,
        config.trajectory.point_count,thinking=backend.thinking,prompt_builder=VisualGoalAboveTrajectoryPrompt(
            speed_mps=config.airsim.speed_mps,planning_interval_s=config.sliding_window.planning_interval_s,
            max_step_m=config.trajectory.max_step_m,max_yaw_step_deg=config.trajectory.max_yaw_step_deg,
            max_yaw_rate_deg_s=config.motion.limits.max_yaw_rate_deg_s))
    checker = OpenAICompatibleTaskCompletionChecker(completion.url,completion.model,completion.api_key,
        completion.timeout_s,prompt_builder=VisualGoalAboveCompletionPrompt(),
        max_completion_tokens=completion.max_completion_tokens,reasoning_effort=completion.reasoning_effort,
        request_format=completion.request_format)
    view = TemporaryView(AirSimCameraView(connection,config.airsim.camera_id))
    with ExitStack() as resources:
        read_connection = connection_factory(config.airsim.host,config.airsim.port,config.airsim.connect_timeout_s)
        resources.callback(read_connection.close_transport)
        base_source = AirSimObservationSource(read_connection,None,config.airsim.camera_id,config.depth.min_m,
            config.depth.max_m,depth_horizontal_fov_deg=config.airsim.depth_horizontal_fov_deg,
            verify_image_pair=True,maximum_pair_time_gap_s=config.airsim.maximum_pair_time_gap_s,
            depth_mapping=config.depth.mapping,camera_mount_position_body_m=config.airsim.camera_mount_position_body_m,
            camera_mount_rpy_deg=config.airsim.camera_mount_rpy_deg)
        source = AboveObservationSource(base_source,read_connection)
        def build_capture_source():
            read = connection_factory(config.airsim.host,config.airsim.port,config.airsim.connect_timeout_s)
            try:
                base = AirSimObservationSource(read,None,config.airsim.camera_id,config.depth.min_m,
                    config.depth.max_m,depth_horizontal_fov_deg=config.airsim.depth_horizontal_fov_deg,
                    verify_image_pair=True,maximum_pair_time_gap_s=config.airsim.maximum_pair_time_gap_s,
                    depth_mapping=config.depth.mapping)
                return AboveObservationSource(base,read)
            except BaseException:
                read.close_transport()
                raise
        stopped_source = AirSimObservationSource(connection,holding.vehicle,config.airsim.camera_id,
            config.depth.min_m,config.depth.max_m,observation_gate=observation_gate)
        stopped_source.capture_worker_factory = build_capture_source
        resources.callback(stopped_source.close_capture_resources)
        flight = AboveFlight(execution,holding,planner,validator,source,config.sliding_window,
            config.motion.limits,config.airsim.speed_mps,config.motion.continuous,settings,
            stopped_source=stopped_source,close_stopped_source=stopped_source.close_capture_resources)
        def trace_factory():
            trace = BackgroundTrace(AboveTrace(settings.output_root))
            trace.event("runtime_configuration",execution_mode="hybrid_sliding_window",
                camera_id=config.airsim.camera_id,goal_model=detector.model,trajectory_model=backend.model,
                completion_model=completion.model,settings=settings,planning_interval_s=config.sliding_window.planning_interval_s)
            return trace
        workflow = VisualGoalAboveWorkflow(flight,view,perception,checker,settings,trace_factory)
        resources.pop_all()
        return workflow
