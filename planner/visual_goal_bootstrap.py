"""Task-only assembly, called explicitly by the common bootstrap."""
from planner.adapters.goal_observer import VLMGoalObserver
from planner.adapters.trajectory_planner import QwenVLPlanner
from planner.adapters.trajectory_planner.prompts.visual_goal_navigation import VisualGoalTrajectoryPrompt
from planner.capabilities.targeting.depth_geometry import DepthGeometry
from planner.capabilities.completion.visual_goal import VisualGoalCompletion
from planner.services.trajectory_safety import TrajectorySafetyChecker
from planner.adapters.diagnostics.visual_goal_trace import VisualGoalTrace
from planner.workflows.visual_goal_navigation import VisualGoalNavigationWorkflow


def build_visual_goal_workflow(config, observation, motion, execution, validator):
    settings = config.visual_goal
    limits = settings.limits
    observer = config.goal_observer
    backend = config.trajectory_planner
    geometry = DepthGeometry(config.depth.min_m, config.depth.max_m, settings.geometry_verified)
    def trace_factory():
        trace = VisualGoalTrace(settings.output_root)
        trace.event("runtime_configuration", goal_model=observer.model, trajectory_model=backend.model,
                    waypoint_fields=("dx", "dy", "dz", "dyaw_deg"), goal_protocol="goal_observation_v1",
                    motion_limits=config.motion.limits, speed_mps=config.airsim.speed_mps,
                    depth_range=(config.depth.min_m, config.depth.max_m),
                    depth_horizontal_fov_deg=settings.depth_horizontal_fov_deg)
        return trace
    return VisualGoalNavigationWorkflow(
        observation,
        VLMGoalObserver(observer.url, observer.model, observer.api_key,
                        min(observer.timeout_s, limits.model_request_timeout_s), observer.thinking),
        QwenVLPlanner(backend.url, backend.model, backend.api_key,
                      min(backend.timeout_s, limits.model_request_timeout_s), config.trajectory.point_count,
                      thinking=backend.thinking, prompt_builder=VisualGoalTrajectoryPrompt()),
        validator, motion, execution, geometry, VisualGoalCompletion(limits),
        TrajectorySafetyChecker(geometry, limits, config.motion.limits, settings.initial_clearance_confirmed),
        limits, trace_factory=trace_factory,
    )
