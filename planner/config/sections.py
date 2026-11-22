"""Dataclass definitions for every configuration section."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any

from ..domain.depth import validate_depth_protocol
from ..domain.depth_foreground import DepthForegroundSettings
from ..domain.view import VIEW_MODES
from ..domain.visual_goal_completion import VisualGoalTriggerRules
from ..domain.visual_goal_limits import VisualGoalLimits


def _section(raw: dict[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key, {})
    if not isinstance(value, dict):
        raise ValueError(f"config section '{key}' must be a mapping")
    return value


@dataclass(frozen=True)
class ProjectAirSimSettings:
    """Connection and scene identity for the ProjectAirSim server."""

    address: str = "127.0.0.1"
    port_topics: int = 8991
    port_services: int = 8992
    timeout_s: float = 10.0
    # Scene config loaded by the client; robot-config files sit beside it.
    scene_file: str = "scene_city_dual.jsonc"
    sim_config_path: str = "config/scenes/city"
    # The dual scene starts with the clock paused; this delay only waits for
    # the renderer so the first calibration screenshot has content.
    delay_after_load_s: float = 2.0
    # Robot names must match the actor names in the scene config.
    rover_name: str = "Rover1"
    drone_name: str = "Drone1"
    # Sensor ids must match the robot config.
    front_camera: str = "FrontCamera"
    chase_camera: str = "Chase"
    down_camera: str = "DownCamera"
    # Streaming views whose camera sets capture-enabled: false. Calibration
    # locates them by elimination instead of waiting for a frame timeout.
    uncapturable_views: tuple[str, ...] = ()
    # Which camera the engine is showing right after a scene loads. The server
    # cycles cameras with ``switch_streaming_view`` and never says which one is
    # current, so the position has to be known some other way. Measuring it at
    # startup costs 60-90 s (screenshots plus a second connection for reference
    # frames) against a 7.8 s simulator, so it is declared here instead. Four
    # cold boots on this machine all came up on ``drone_front``. If a new map or
    # build starts somewhere else, every view button is off by the same number
    # of steps -- run scripts/calibrate_streaming_views.py and change this line.
    view_cycle_start: str = "drone_front"

    def __post_init__(self) -> None:
        if not str(self.address).strip():
            raise ValueError("projectairsim.address must be non-empty")
        for name in ("port_topics", "port_services"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f"projectairsim.{name} must be in [1, 65535]")
        if self.timeout_s <= 0:
            raise ValueError("projectairsim.timeout_s must be positive")
        # Scene identity is not validated here: it arrives from the map entry at
        # session build time, and an entry without a scene belongs to a family
        # started straight from a binary.
        if self.delay_after_load_s < 0:
            raise ValueError("projectairsim.delay_after_load_s must be >= 0")
        for name in (
            "rover_name",
            "drone_name",
            "front_camera",
            "chase_camera",
            "down_camera",
        ):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"projectairsim.{name} must be non-empty")
        views = tuple(self.uncapturable_views)
        if len(set(views)) != len(views):
            raise ValueError(
                "projectairsim.uncapturable_views must not repeat a view"
            )
        unknown = [view for view in views if view not in VIEW_MODES]
        if unknown:
            raise ValueError(
                "projectairsim.uncapturable_views has unknown views: "
                + ", ".join(unknown)
            )
        if self.view_cycle_start not in VIEW_MODES:
            raise ValueError(
                f"projectairsim.view_cycle_start must be one of "
                f"{', '.join(VIEW_MODES)}, got {self.view_cycle_start!r}"
            )


@dataclass(frozen=True)
class DroneSettings:
    """``drone`` section: UAV flight envelope, timeouts and camera mount."""

    speed_mps: float = 2.0
    move_timeout_s: float = 60.0
    takeoff_timeout_s: float = 30.0
    land_timeout_s: float = 30.0
    readiness_stable_s: float = 2.0
    readiness_poll_interval_s: float = 0.25
    #: 原生起飞结束后再爬升的高度，米。原生 takeoff 只把飞机抬离地面一点，
    #: 视觉任务需要更高的作业高度，这一项补上差额；取零表示不补。
    takeoff_climb_m: float = 1.0
    # FrontCamera mount in the body frame (forward, right, down; metres) and
    # its NED roll/pitch/yaw orientation. The reference UAV mount is level and
    # one metre ahead of the body origin.
    camera_offset_m: tuple[float, float, float] = (1.0, 0.0, 0.0)
    camera_mount_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        for name in (
            "speed_mps",
            "move_timeout_s",
            "takeoff_timeout_s",
            "land_timeout_s",
            "readiness_poll_interval_s",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"drone.{name} must be finite and positive")
        for name in ("readiness_stable_s", "takeoff_climb_m"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(f"drone.{name} must be finite and non-negative")
        for name in ("camera_offset_m", "camera_mount_rpy_deg"):
            values = getattr(self, name)
            if len(values) != 3 or not all(
                not isinstance(value, bool)
                and isinstance(value, (int, float))
                and math.isfinite(float(value))
                for value in values
            ):
                raise ValueError(
                    f"drone.{name} must contain three finite numbers"
                )

    def camera(self, base: CameraSettings) -> CameraSettings:
        """Use the drone's level, forward camera mount instead of the rover's."""
        return replace(
            base,
            offset_m=self.camera_offset_m,
            mount_rpy_deg=self.camera_mount_rpy_deg,
        )


@dataclass(frozen=True)
class CameraSettings:
    width: int = 1280
    height: int = 720
    hfov_deg: float = 90.0
    # ProjectAirSim depth (image-type 2) is metric ray range in metres,
    # saturated to 65504 m on the sky; far_m is kept for config validation
    # only and does not participate in decoding.
    far_m: float = 1000.0
    depth_is_range: bool = True
    # Mount offset in the body frame: forward, right, down (metres), matching
    # the planner's NED body convention and the robot jsonc sensor origin.
    offset_m: tuple[float, float, float] = (1.8, 0.0, -0.8)
    # Camera mount orientation relative to the body (degrees, NED roll/pitch/yaw),
    # used to recover the body pose from the camera pose published on topics.
    mount_rpy_deg: tuple[float, float, float] = (0.0, -3.0, 0.0)


@dataclass(frozen=True)
class TargetDetectionSettings:
    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 30.0
    reasoning_effort: str = "none"
    request_format: str = "openai"
    max_completion_tokens: int = 2048
    max_image_side: int = 1024
    minimum_depth_m: float = 0.5

    def __post_init__(self) -> None:
        if not self.timeout_s > 0:
            raise ValueError("target_detection.timeout_s must be positive")
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid target_detection.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "target_detection.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "target_detection.anthropic requires reasoning_effort 'none'"
            )
        if (
            type(self.max_completion_tokens) is not int
            or not 1 <= self.max_completion_tokens <= 32768
        ):
            raise ValueError(
                "target_detection.max_completion_tokens must be in [1, 32768]"
            )
        if (
            type(self.max_image_side) is not int
            or not 32 <= self.max_image_side <= 8192
        ):
            raise ValueError(
                "target_detection.max_image_side must be in [32, 8192]"
            )
        if not self.minimum_depth_m > 0:
            raise ValueError("target_detection.minimum_depth_m must be positive")


@dataclass(frozen=True)
class TrajectoryPlannerSettings:
    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 30.0
    max_tokens: int = 256
    reasoning_effort: str = "none"
    request_format: str = "openai"
    point_count: int = 5
    max_step_m: float = 2.0
    max_yaw_step_deg: float = 45.0

    def __post_init__(self) -> None:
        if self.point_count != 5:
            raise ValueError("the trajectory protocol requires exactly 5 points")
        for name in ("max_step_m", "max_yaw_step_deg"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0:
                raise ValueError(f"trajectory_planner.{name} must be positive")
        if self.max_yaw_step_deg > 180:
            raise ValueError(
                "trajectory_planner.max_yaw_step_deg must not exceed 180"
            )
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid trajectory_planner.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "trajectory_planner.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "trajectory_planner.anthropic requires reasoning_effort 'none'"
            )


@dataclass(frozen=True)
class DepthSettings:
    mapping: str = "log_inverse_gray_v1"
    min_m: float = 0.5
    max_m: float = 200.0

    def __post_init__(self) -> None:
        validate_depth_protocol(self.mapping, self.min_m, self.max_m)


@dataclass(frozen=True)
class TaskCompletionSettings:
    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 20.0
    max_completion_tokens: int = 2048
    reasoning_effort: str = "none"
    request_format: str = "openai"

    def __post_init__(self) -> None:
        if self.timeout_s <= 0:
            raise ValueError("task_completion.timeout_s must be positive")
        if self.max_completion_tokens < 1:
            raise ValueError(
                "task_completion.max_completion_tokens must be a positive integer"
            )
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid task_completion.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "task_completion.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "task_completion.anthropic requires reasoning_effort 'none'"
            )


@dataclass(frozen=True)
class TaskParserSettings:
    """Natural-language mission parser (``task_parser`` section)."""

    url: str = ""
    model: str = ""
    # Inline key (takes precedence); empty falls back to api_key_env.
    api_key: str = ""
    api_key_env: str = "VLN_API_KEY"
    timeout_s: float = 60.0
    max_completion_tokens: int = 2048
    reasoning_effort: str = "none"
    request_format: str = "openai"

    def __post_init__(self) -> None:
        if not self.timeout_s > 0:
            raise ValueError("task_parser.timeout_s must be positive")
        if (
            type(self.max_completion_tokens) is not int
            or not 1 <= self.max_completion_tokens <= 32768
        ):
            raise ValueError(
                "task_parser.max_completion_tokens must be in [1, 32768]"
            )
        if self.reasoning_effort not in {"none", "low", "medium", "high"}:
            raise ValueError("invalid task_parser.reasoning_effort")
        if self.request_format not in ("openai", "deepseek", "anthropic"):
            raise ValueError(
                "task_parser.request_format must be openai, deepseek or anthropic"
            )
        if self.request_format == "anthropic" and self.reasoning_effort != "none":
            raise ValueError(
                "task_parser.anthropic requires reasoning_effort 'none'"
            )


@dataclass(frozen=True)
class VisualGoalNavigationSettings:
    enabled: bool = True
    geometry_verified: bool = False
    collision_check_enabled: bool = False
    output_root: str = "output/visual_goal"
    limits: VisualGoalLimits = field(default_factory=VisualGoalLimits)
    completion_rules_file: str | None = None
    completion_rules: VisualGoalTriggerRules = field(
        default_factory=VisualGoalTriggerRules
    )
    depth_foreground_filter: DepthForegroundSettings = field(
        default_factory=DepthForegroundSettings
    )

    def __post_init__(self) -> None:
        for name in ("enabled", "geometry_verified", "collision_check_enabled"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be boolean")
        if not isinstance(self.output_root, str) or not self.output_root.strip():
            raise ValueError("visual goal output_root must be non-empty text")
        if self.completion_rules_file is not None and (
            not isinstance(self.completion_rules_file, str)
            or not self.completion_rules_file.strip()
        ):
            raise ValueError("completion_rules_file must be non-empty text")


@dataclass(frozen=True)
class FreeNavigationSettings:
    """``free_navigation`` section: native-contact recovery knobs."""

    collision_backoff_m: float = 3.0
    maximum_collision_recoveries: int = 3
    # Sustained forward progress that clears the consecutive recovery count.
    collision_recovery_reset_m: float = 20.0

    def __post_init__(self) -> None:
        for name in ("collision_backoff_m", "collision_recovery_reset_m"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if (
            type(self.maximum_collision_recoveries) is not int
            or self.maximum_collision_recoveries < 1
        ):
            raise ValueError(
                "maximum_collision_recoveries must be a positive integer"
            )


@dataclass(frozen=True)
class ServerSettings:
    """HTTP server and in-process worker knobs (``server`` section)."""

    host: str = "127.0.0.1"
    port: int = 8000
    detector: str = "manual"
    planner: str = "geometric"
    manual_bbox: tuple[float, float, float, float] = (0.45, 0.82, 0.55, 0.92)
    manual_surface_points: tuple[tuple[float, float], ...] = ()
    allowed_origins: tuple[str, ...] = ("*",)
    log_buffer_size: int = 500
    reconnect_interval_s: float = 2.0
    startup_timeout_s: float = 60.0
    video_fps: float = 10.0
    # Vehicle ids the goal router accepts; every task names exactly one.
    vehicles: tuple[str, ...] = ("rover", "drone")
    default_vehicle: str = "rover"

    def __post_init__(self) -> None:
        if not 1 <= self.port <= 65535:
            raise ValueError("server.port must be in [1, 65535]")
        if self.detector not in ("manual", "vlm"):
            raise ValueError("server.detector must be 'manual' or 'vlm'")
        if self.planner not in ("geometric", "vlm"):
            raise ValueError("server.planner must be 'geometric' or 'vlm'")
        box = self.manual_bbox
        if len(box) != 4 or not (
            0.0 <= box[0] < box[2] <= 1.0 and 0.0 <= box[1] < box[3] <= 1.0
        ):
            raise ValueError(
                "server.manual_bbox must be ordered normalized "
                "xmin,ymin,xmax,ymax"
            )
        if len(self.manual_surface_points) > 5:
            raise ValueError("server.manual_surface_points allows at most 5 points")
        if not self.allowed_origins:
            raise ValueError("server.allowed_origins must not be empty")
        if self.log_buffer_size <= 0:
            raise ValueError("server.log_buffer_size must be positive")
        if self.reconnect_interval_s <= 0:
            raise ValueError("server.reconnect_interval_s must be positive")
        if self.startup_timeout_s <= 0:
            raise ValueError("server.startup_timeout_s must be positive")
        if not 0.0 < self.video_fps <= 60.0:
            raise ValueError("server.video_fps must be in (0, 60]")
        if not self.vehicles or any(
            not str(vehicle).strip() for vehicle in self.vehicles
        ):
            raise ValueError("server.vehicles must contain non-empty ids")
        if len(set(self.vehicles)) != len(self.vehicles):
            raise ValueError("server.vehicles must be unique")
        if self.default_vehicle not in self.vehicles:
            raise ValueError(
                "server.default_vehicle must appear in server.vehicles"
            )


@dataclass(frozen=True)
class SimulatorSettings:
    """``simulator`` section: process ownership for map switching."""

    display: str = ":104"
    runtime_dir: str = "output/xpra"
    map_catalog_file: str = "maps.yaml"
    map_marker_file: str = "output/xpra/current_map"
    launch_timeout_s: float = 180.0
    stop_timeout_s: float = 40.0
    port_release_timeout_s: float = 30.0
    restart_attempts: int = 2

    def __post_init__(self) -> None:
        for name in ("runtime_dir",
                     "map_catalog_file", "map_marker_file"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"simulator.{name} must be non-empty text")
        if not isinstance(self.display, str) or not (
            self.display.startswith(":") and self.display[1:].isdigit()
        ):
            raise ValueError("simulator.display must look like ':104'")
        for name in ("launch_timeout_s", "stop_timeout_s",
                     "port_release_timeout_s"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"simulator.{name} must be finite and positive")
        if type(self.restart_attempts) is not int or self.restart_attempts < 1:
            raise ValueError("simulator.restart_attempts must be a positive integer")


def _triple(values: Any, *, where: str) -> tuple[float, float, float]:
    triple = tuple(float(value) for value in values)
    if len(triple) != 3:
        raise ValueError(f"{where} must contain three numbers")
    return triple
