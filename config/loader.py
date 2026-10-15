from __future__ import annotations

import os
import math
from pathlib import Path
from dataclasses import fields

import yaml

from planner.errors import ConfigurationError
from planner.domain.motion import MotionLimits
from planner.domain.sliding_window import SlidingWindowSettings
from .schema import MotionConfig, ProgressReviewConfig
from .validation import number, integer, mapping, string
from planner.domain.depth import validate_depth_protocol
from .target_detection import target_detection_config
from .visual_goal import visual_goal_config
from .basic_motion import basic_motion_config
from .task_completion import completion_config
from .visual_goal_completion import completion_rules
from .schema import AirSimConfig, ApiConfig, DepthConfig, PlannerConfig, RecordingConfig, TrajectoryConfig, TaskParserConfig


def load_config(path: str | Path) -> PlannerConfig:
    config_path = Path(path).expanduser().resolve()
    try:
        raw = mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}, "config",
                      {"task_parser", "trajectory_planner", "airsim", "depth", "trajectory", "recording",
                       "basic_motion", "motion", "visual_goal_navigation", "task_completion", "sliding_window", "navigation", "target_detection", "progress_review"})
        if "navigation" in raw:
            raise ConfigurationError("legacy navigation workflow removed: remove navigation settings and configure basic_motion")
        parser = raw["task_parser"]
        planning = raw["trajectory_planner"]
        sim = mapping(raw["airsim"], "airsim", {f.name for f in fields(AirSimConfig)})
        depth = mapping(raw["depth"], "depth", {f.name for f in fields(DepthConfig)})
        trajectory = mapping(raw["trajectory"], "trajectory", {f.name for f in fields(TrajectoryConfig)})
        recording = mapping(raw.get("recording", {}), "recording", {f.name for f in fields(RecordingConfig)})
        visual_raw = raw.get("visual_goal_navigation", {})
        visual_goal = visual_goal_config(visual_raw)
        if visual_goal.enabled:
            visual_goal = visual_goal_config(visual_raw, completion_rules=completion_rules(
                config_path, visual_goal.completion_rules_file))
        review = mapping(raw.get("progress_review", {}), "progress_review", {"enabled"})
        if type(review.get("enabled", False)) is not bool:
            raise ConfigurationError("progress_review.enabled must be boolean")
        if review.get("enabled", False):
            raise ConfigurationError("progress_review is reserved; no workflow review strategy has been implemented")
        result = PlannerConfig(
            task_parser=_task_parser_config(parser),
            trajectory_planner=_api_config(planning, require_credentials=False),
            airsim=AirSimConfig(
                host=string(sim.get("host", ""), "airsim.host", allow_empty=True),
                port=integer(sim["port"], "airsim.port", 1, 65535), camera_id=string(sim["camera_id"], "airsim.camera_id"),
                connect_timeout_s=number(sim["connect_timeout_s"], "airsim.connect_timeout_s"),
                move_timeout_s=number(sim["move_timeout_s"], "airsim.move_timeout_s"), speed_mps=number(sim["speed_mps"], "airsim.speed_mps"),
                depth_horizontal_fov_deg=number(sim["depth_horizontal_fov_deg"], "airsim.depth_horizontal_fov_deg") if sim.get("depth_horizontal_fov_deg") is not None else None,
                maximum_pair_time_gap_s=number(sim.get("maximum_pair_time_gap_s", .05), "airsim.maximum_pair_time_gap_s", positive=False),
            ),
            depth=DepthConfig(number(depth["min_m"], "depth.min_m", positive=False), number(depth["max_m"], "depth.max_m"), depth.get("mapping", "log_inverse_gray_v1")),
            trajectory=TrajectoryConfig(
                integer(trajectory["point_count"], "trajectory.point_count (must be exactly 5)", 5, 5), number(trajectory["max_step_m"], "trajectory.max_step_m"),
                number(trajectory["max_yaw_step_deg"], "trajectory.max_yaw_step_deg"),
            ),
            recording=RecordingConfig(
                string(recording.get("output_root", "output/camera_api"), "recording.output_root"),
                number(recording.get("fps", 5.0), "recording.fps"),
            ),
            basic_motion=basic_motion_config(raw.get("basic_motion", {})),
            motion=_motion_config(raw.get("motion", {})),
            visual_goal=visual_goal,
            task_completion=completion_config(raw.get("task_completion", {})),
            sliding_window=_sliding_window_config(raw.get("sliding_window", {})),
            target_detection=target_detection_config(raw["target_detection"]) if "target_detection" in raw else None,
        )
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ConfigurationError(f"invalid config {config_path}: {exc}") from exc
    if result.airsim.depth_horizontal_fov_deg is not None and result.airsim.depth_horizontal_fov_deg >= 180:
        raise ConfigurationError("airsim.depth_horizontal_fov_deg must be below 180")
    if result.trajectory.point_count != 5:
        raise ConfigurationError("trajectory.point_count must be exactly 5 for model planning")
    if result.airsim.speed_mps <= 0:
        raise ConfigurationError("speed_mps must be positive")
    for name, value in (
        ("speed_mps", result.airsim.speed_mps),
        ("connect_timeout_s", result.airsim.connect_timeout_s),
        ("move_timeout_s", result.airsim.move_timeout_s),
        ("recording.fps", result.recording.fps),
        ("trajectory.max_step_m", result.trajectory.max_step_m),
        ("trajectory.max_yaw_step_deg", result.trajectory.max_yaw_step_deg),
    ):
        if not math.isfinite(value) or value <= 0:
            raise ConfigurationError(f"{name} must be finite and positive")
    try:
        validate_depth_protocol(result.depth.mapping, result.depth.min_m, result.depth.max_m)
    except ValueError as exc:
        raise ConfigurationError(str(exc)) from exc
    if result.recording.fps <= 0:
        raise ConfigurationError("recording fps must be positive")
    if result.trajectory.max_yaw_step_deg > 180:
        raise ConfigurationError("trajectory.max_yaw_step_deg must not exceed 180")
    if result.basic_motion.free_space.clearance_radius_m <= result.motion.limits.position_tolerance_m:
        raise ConfigurationError("basic_motion clearance radius must include body size beyond position tolerance")
    return result


def _sliding_window_config(raw: dict) -> SlidingWindowSettings:
    if not isinstance(raw, dict):
        raise ConfigurationError("sliding_window must be a mapping")
    names = {field.name for field in fields(SlidingWindowSettings)}
    unknown = set(raw) - names
    if unknown:
        raise ConfigurationError(f"unknown sliding_window settings: {sorted(unknown)}")
    return SlidingWindowSettings(**raw)


def _api_config(raw: dict, *, require_credentials: bool = True, extra_fields: set[str] = frozenset()) -> ApiConfig:
    mapping(raw, "API settings", {f.name for f in fields(ApiConfig)} | {"api_key_env", "require_api_key"} | extra_fields)
    if not isinstance(raw, dict):
        raise ConfigurationError("API settings must be a mapping")
    env_name = string(raw.get("api_key_env", ""), "api_key_env", allow_empty=True).strip()
    api_key = (os.environ.get(env_name, "") if env_name else "") or string(raw.get("api_key") or "", "api_key", allow_empty=True)
    thinking = raw.get("thinking")
    if thinking not in (None, "enabled", "disabled"):
        raise ConfigurationError("thinking must be enabled or disabled")
    if type(raw.get("require_api_key", False)) is not bool:
        raise ConfigurationError("require_api_key must be boolean")
    if require_credentials and raw.get("require_api_key") and (not api_key.strip() or api_key == "no-key"):
        raise ConfigurationError("missing API key: fill trajectory_planner.api_key in the selected config")
    timeout = number(raw["timeout_s"], "API timeout_s")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ConfigurationError("API timeout_s must be finite and positive")
    return ApiConfig(string(raw["url"], "API url"), string(raw["model"], "API model"), api_key or "no-key", timeout, thinking, require_api_key=raw.get("require_api_key", False))


def _task_parser_config(raw: dict) -> TaskParserConfig:
    backend = _api_config(raw, extra_fields={"max_completion_tokens", "reasoning_effort", "request_format"})
    tokens = raw.get("max_completion_tokens")
    effort = raw.get("reasoning_effort")
    if tokens is not None and (isinstance(tokens, bool) or not isinstance(tokens, int) or tokens <= 0):
        raise ConfigurationError("task_parser.max_completion_tokens must be a positive integer")
    if effort is not None and (not isinstance(effort, str) or effort not in {"none", "low", "medium", "high", "xhigh", "max"}):
        raise ConfigurationError("invalid task_parser.reasoning_effort")
    request_format = raw.get("request_format", "openai")
    if request_format not in ("openai", "deepseek"):
        raise ConfigurationError("task_parser.request_format must be openai or deepseek")
    return TaskParserConfig(backend.url, backend.model, backend.api_key, backend.timeout_s,
                            backend.thinking, tokens, effort, request_format)


def _motion_config(raw: dict) -> MotionConfig:
    if not isinstance(raw, dict):
        raise ConfigurationError("motion must be a mapping")
    names = {field.name for field in fields(MotionLimits)}
    unknown = set(raw) - names
    if unknown:
        raise ConfigurationError(f"unknown motion settings: {sorted(unknown)}")
    limits = MotionLimits(**{name: number(raw[name], "motion."+name) for name in names if name in raw})
    return MotionConfig(limits)

