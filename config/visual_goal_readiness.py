"""Validate only the capabilities requested by this visual-goal consumer."""
from planner.errors import ConfigurationError


def validate_visual_goal_readiness(config):
    settings = config.visual_goal
    planning = config.trajectory_planner
    if planning.require_api_key and (not planning.api_key.strip() or planning.api_key == "no-key"):
        raise ConfigurationError("visual goal requires trajectory_planner API key")
    backend = config.target_detection
    if backend is None or not backend.api_key.strip() or backend.api_key == "no-key":
        raise ConfigurationError("visual goal requires target_detection and TARGET_DETECTION_API_KEY")
    if config.airsim.depth_horizontal_fov_deg is None:
        raise ConfigurationError("visual goal requires calibrated airsim.depth_horizontal_fov_deg")
    if not settings.geometry_verified:
        raise ConfigurationError("visual goal requires verified RGB/depth geometry")
    try:
        settings.limits.validate_arrival_radius(settings.completion_rules.radius_m, config.depth.max_m)
    except ValueError as exc:
        raise ConfigurationError(str(exc)) from exc
    if not config.task_completion.api_key.strip() or config.task_completion.api_key == "no-key":
        raise ConfigurationError("visual goal requires TASK_COMPLETION_API_KEY for semantic completion")
