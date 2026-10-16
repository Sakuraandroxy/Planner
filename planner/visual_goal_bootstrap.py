"""Task-only assembly, called explicitly by the common bootstrap."""
from contextlib import ExitStack
from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.prompts.visual_goal_navigation import VisualGoalTrajectoryPrompt
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.approach import TargetApproach
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.adapters.diagnostics.visual_goal_trace import VisualGoalTrace
from planner.workflows.visual_goal_navigation import VisualGoalNavigationWorkflow
from planner.adapters.task_completion import OpenAICompatibleTaskCompletionChecker
from planner.adapters.task_completion.prompts import VisualGoalCompletionPrompt
from config.visual_goal_readiness import validate_visual_goal_readiness
from planner.adapters.multimodal_chat import MultimodalChatClient
from planner.adapters.target_detection import OpenAICompatibleTargetDetector
from planner.capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from planner.adapters.airsim import AirSimConnection, AirSimObservationSource


def build_visual_goal_workflow(config, observation, motion, execution, validator, *, connection_factory=AirSimConnection):
    validate_visual_goal_readiness(config)
    settings = config.visual_goal
    limits = settings.limits
    observer = config.target_detection
    backend = config.trajectory_planner
    completion_backend = config.task_completion
    geometry = DepthGeometry(config.depth.min_m, config.depth.max_m, settings.geometry_verified)
    transport = MultimodalChatClient(observer.url, observer.model, observer.api_key, observer.timeout_s,
        max_completion_tokens=observer.max_completion_tokens, reasoning_effort=observer.reasoning_effort,
        request_format=observer.request_format)
    detector = OpenAICompatibleTargetDetector(transport, maximum_image_side=observer.max_image_side)
    goal_observer = BBoxGoalObserver(detector, min(observer.timeout_s, limits.model_request_timeout_s))
    def trace_factory():
        trace = VisualGoalTrace(settings.output_root)
        trace.event("runtime_configuration", goal_model=observer.model, trajectory_model=backend.model,
                    waypoint_fields=("dx", "dy", "dz", "dyaw_deg"), goal_protocol="normalized_bbox_corners_v1",
                    target_point_source="first_anchor_bounded_target_memory", execution_policy="direct_model_trajectory",
                    motion_limits=config.motion.limits, speed_mps=config.airsim.speed_mps,
                    depth_range=(config.depth.min_m, config.depth.max_m),
                    depth_horizontal_fov_deg=config.airsim.depth_horizontal_fov_deg,
                    execution_mode="continuous_world_queue" if config.sliding_window.enabled else "stopped_segments",
                    planning_interval_s=config.sliding_window.planning_interval_s,
                    max_buffer_points=config.sliding_window.max_buffer_points,
                    target_refresh_interval_s=limits.target_refresh_interval_s,
                    target_position_update_radius_m=limits.target_position_update_radius_m,
                    camera_mount_position_body_m=config.airsim.camera_mount_position_body_m,
                    camera_mount_rpy_deg=config.airsim.camera_mount_rpy_deg,
                    target_depth_sampling_mode="direct_pixel")
        return trace
    approach = TargetApproach()
    with ExitStack() as resources:
        moving_source = None
        if config.sliding_window.enabled:
            # Dedicated read-only transport: it receives no controller or gate.
            read_connection = connection_factory(config.airsim.host, config.airsim.port, config.airsim.connect_timeout_s)
            resources.callback(read_connection.close_transport)
            moving_source = AirSimObservationSource(read_connection, None, config.airsim.camera_id,
                config.depth.min_m, config.depth.max_m, verify_image_pair=True,
                depth_horizontal_fov_deg=config.airsim.depth_horizontal_fov_deg,
                maximum_pair_time_gap_s=config.airsim.maximum_pair_time_gap_s,
                depth_mapping=config.depth.mapping,
                camera_mount_position_body_m=config.airsim.camera_mount_position_body_m,
                camera_mount_rpy_deg=config.airsim.camera_mount_rpy_deg)
        workflow = VisualGoalNavigationWorkflow(
            observation,
            goal_observer,
            QwenVLPlanner(backend.url, backend.model, backend.api_key,
                          min(backend.timeout_s, limits.model_request_timeout_s), config.trajectory.point_count,
                          thinking=backend.thinking, prompt_builder=VisualGoalTrajectoryPrompt()),
            validator, motion, execution, geometry, VisualGoalCompletion(limits, approach),
            limits, trace_factory=trace_factory,
            completion_checker=OpenAICompatibleTaskCompletionChecker(completion_backend.url, completion_backend.model,
                completion_backend.api_key, completion_backend.timeout_s, prompt_builder=VisualGoalCompletionPrompt(),
                max_completion_tokens=completion_backend.max_completion_tokens,
                reasoning_effort=completion_backend.reasoning_effort, request_format=completion_backend.request_format),
            trigger_rules=settings.completion_rules,
            sliding_window=config.sliding_window,
            motion_limits=config.motion.limits,
            collision_check=execution.vehicle.collision_state if settings.collision_check_enabled else None,
            moving_observation_source=moving_source, stream_speed_mps=config.airsim.speed_mps,
            stream_waypoint_timeout_s=config.airsim.move_timeout_s,
        )
        resources.pop_all()  # workflow now owns the read-only transport
        return workflow
