"""Configuration loading for the ProjectAirSim VLN skeleton."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import yaml

from ..domain.depth_foreground import DepthForegroundSettings
from ..domain.motion import ContinuousTrackingSettings
from ..domain.engine_family import FamilyCatalog, parse_family_catalog
from ..domain.sliding_window import SlidingWindowSettings
from ..domain.visual_goal_completion import VisualGoalTriggerRules
from ..domain.visual_goal_limits import VisualGoalLimits
from ..domain.motion import MotionLimits
from .maps import (
    MapCatalog,
    MapEntry,
    MapScene,
    _flatten_scene_defaults,
    load_map_catalog,
)
from .profile import (
    PROFILE_SWITCHABLE_SECTIONS,
    ProfileInfo,
    _profile_info,
    profile_catalog,
    profile_drift,
)
from .sections import (
    CameraSettings,
    DepthSettings,
    DroneSettings,
    FreeNavigationSettings,
    ProjectAirSimSettings,
    ServerSettings,
    SimulatorSettings,
    TargetDetectionSettings,
    TaskCompletionSettings,
    TaskParserSettings,
    TrajectoryPlannerSettings,
    VisualGoalNavigationSettings,
    _section,
    _triple,
)
from .vehicles import (
    VehicleModelCatalog,
    VehicleModelGroup,
    VehicleModelOption,
    _family_vehicle_models,
    default_vehicle_models,
    load_vehicle_models,
)
from .paths import (
    _project_root,
    available_profiles,
    default_config_path,
    default_profile,
    profile_config_path,
    profile_of,
    profiles_root,
    shared_or_profile,
)


@dataclass(frozen=True)
class AppConfig:
    projectairsim: ProjectAirSimSettings
    camera: CameraSettings
    motion: MotionLimits
    maps: MapCatalog
    families: FamilyCatalog = field(default_factory=lambda: FamilyCatalog(families=()))
    vehicle_models: VehicleModelCatalog = field(
        default_factory=default_vehicle_models
    )
    simulator: SimulatorSettings = field(default_factory=SimulatorSettings)
    drone: DroneSettings = field(default_factory=DroneSettings)
    target_detection: TargetDetectionSettings = field(
        default_factory=TargetDetectionSettings
    )
    trajectory_planner: TrajectoryPlannerSettings = field(
        default_factory=TrajectoryPlannerSettings
    )
    server: ServerSettings = field(default_factory=ServerSettings)
    depth: DepthSettings = field(default_factory=DepthSettings)
    continuous: ContinuousTrackingSettings = field(
        default_factory=ContinuousTrackingSettings
    )
    sliding_window: SlidingWindowSettings = field(
        default_factory=SlidingWindowSettings
    )
    task_completion: TaskCompletionSettings = field(
        default_factory=TaskCompletionSettings
    )
    task_parser: TaskParserSettings = field(
        default_factory=TaskParserSettings
    )
    visual_goal_navigation: VisualGoalNavigationSettings = field(
        default_factory=VisualGoalNavigationSettings
    )
    #: Where this configuration was read from. A mission child needs it verbatim:
    #: the child is a separate process, and re-deriving the path from a repository
    #: layout would let a service started with ``--config somewhere/else.yaml`` hand
    #: its children a config it never asked for.
    path: str = ""
    free_navigation: FreeNavigationSettings = field(
        default_factory=FreeNavigationSettings
    )
    #: How this profile introduces itself on the console.
    profile: ProfileInfo = field(default_factory=ProfileInfo)


def build_camera_settings(raw: dict[str, Any]) -> CameraSettings:
    """Camera settings a simulator family declares (``camera`` in simulators.yaml).

    Resolution, field of view, mount geometry and depth encoding are properties
    of the simulator package that produces the images, so they sit with the
    family instead of the navigation core.
    """
    known = {
        "width",
        "height",
        "hfov_deg",
        "far_m",
        "depth_is_range",
        "offset_m",
        "mount_rpy_deg",
        "drone_offset_m",
        "drone_mount_rpy_deg",
    }
    unknown = set(raw) - known
    if unknown:
        raise ValueError(
            f"simulator camera has unknown settings {sorted(unknown)}; "
            f"expected any of {sorted(known)}"
        )
    camera = CameraSettings(
        width=int(raw.get("width", 1280)),
        height=int(raw.get("height", 720)),
        hfov_deg=float(raw.get("hfov_deg", 90.0)),
        far_m=float(raw.get("far_m", 1000.0)),
        depth_is_range=bool(raw.get("depth_is_range", True)),
        offset_m=_triple(raw.get("offset_m", (1.8, 0.0, -0.8)), where="camera.offset_m"),
        mount_rpy_deg=_triple(
            raw.get("mount_rpy_deg", (0.0, -3.0, 0.0)),
            where="camera.mount_rpy_deg",
        ),
    )
    if camera.width <= 0 or camera.height <= 0:
        raise ValueError("camera dimensions must be positive")
    if not 0.0 < camera.hfov_deg < 180.0:
        raise ValueError("camera.hfov_deg must be in (0, 180)")
    if camera.far_m <= 0:
        raise ValueError("camera.far_m must be positive")
    return camera


#: Sections a profile file may declare. Everything else belongs to app.yml
#: (program), simulators.yaml (family) or maps.yaml (map); listing them keeps a
#: misplaced section from being silently ignored.
PROFILE_SECTIONS = PROFILE_SWITCHABLE_SECTIONS | {"app", "simulator_families"}


def load_config(path: str | Path) -> AppConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError("config root must be a mapping")
    unknown = set(raw) - PROFILE_SECTIONS
    if unknown:
        raise ValueError(
            f"profile config has sections that do not belong to a profile: "
            f"{sorted(unknown)}; server and process live in app.yml, "
            "vehicle_models and camera in simulators.yaml, scene identity in maps.yaml"
        )

    motion_raw = _section(raw, "motion")

    families_path = shared_or_profile(path, "simulators.yaml")
    families_raw = _section(raw, "simulator_families")
    if not families_raw and families_path.is_file():
        families_raw = yaml.safe_load(families_path.read_text(encoding="utf-8")) or {}
    families = parse_family_catalog(families_raw)
    default_family = families.require(families.default_family)

    # Program-level settings shared by every profile: the HTTP service itself and
    # who owns the simulator processes. Neither is tied to a map or a policy.
    app_path = shared_or_profile(path, "app.yml")
    app_raw = _section(raw, "app")
    if not app_raw and app_path.is_file():
        app_raw = yaml.safe_load(app_path.read_text(encoding="utf-8")) or {}
    if not isinstance(app_raw, dict):
        raise ValueError("app.yml root must be a mapping")

    # How to reach the simulator comes from the family that owns the package.
    # Scene identity is per map and lives on the catalog entry; the legacy
    # ``scene_*`` keys below are only read for a config that carries no catalog.
    # How to reach the simulator: one family, one set of ports. The client and the
    # readiness probes therefore read the same numbers.
    projectairsim = ProjectAirSimSettings(
        address="127.0.0.1",
        port_topics=default_family.ports.get("topics", 8991),
        port_services=default_family.ports.get("services", 8992),
        timeout_s=default_family.timeout_s or 10.0,
    )

    # Who owns the simulator processes describes this machine, not a map: it sits
    # with the families instead of the navigation core.
    simulator_raw = _section(app_raw, "process")
    simulator = SimulatorSettings(
        display=str(simulator_raw.get("display", ":104")),
        runtime_dir=str(simulator_raw.get("runtime_dir", "output/xpra")),
        map_catalog_file=str(
            simulator_raw.get("map_catalog_file", "maps.yaml")
        ),
        map_marker_file=str(
            simulator_raw.get("map_marker_file", "output/xpra/current_map")
        ),
        launch_timeout_s=float(simulator_raw.get("launch_timeout_s", 180.0)),
        stop_timeout_s=float(simulator_raw.get("stop_timeout_s", 40.0)),
        port_release_timeout_s=float(
            simulator_raw.get("port_release_timeout_s", 30.0)
        ),
        restart_attempts=int(simulator_raw.get("restart_attempts", 2)),
    )
    maps = load_map_catalog(
        shared_or_profile(path, simulator.map_catalog_file),
        default_simulator=families.default_family,
        # Scene conventions belong to the simulator package; each map entry writes
        # only what it changes.
        scene_defaults={
            family.id: _flatten_scene_defaults(
                family.scene_defaults, simulator=family.id
            )
            for family in families.families
        },
        fallback=MapEntry(
            id="default",
            label="默认地图",
            simulator=default_family.id,
            # Without a catalog there is no scene either: a deployment that wants
            # to run a session ships maps.yaml.
            scene=MapScene(),
        ),
    )

    unknown = set(maps.simulator_ids()) - set(families.ids)
    if unknown:
        raise ValueError(
            f"maps reference simulators that are not configured: {sorted(unknown)}; "
            f"configured simulators are {list(families.ids)}"
        )

    camera = build_camera_settings(default_family.camera)

    drone_raw = _section(raw, "drone")
    drone_offset = _triple(
        default_family.camera.get("drone_offset_m", (1.0, 0.0, 0.0)),
        where="camera.drone_offset_m in simulators.yaml",
    )
    drone_mount = _triple(
        default_family.camera.get("drone_mount_rpy_deg", (0.0, 0.0, 0.0)),
        where="camera.drone_mount_rpy_deg in simulators.yaml",
    )
    drone = DroneSettings(
        speed_mps=float(drone_raw.get("speed_mps", 2.0)),
        move_timeout_s=float(drone_raw.get("move_timeout_s", 60.0)),
        takeoff_timeout_s=float(drone_raw.get("takeoff_timeout_s", 30.0)),
        land_timeout_s=float(drone_raw.get("land_timeout_s", 30.0)),
        readiness_stable_s=float(drone_raw.get("readiness_stable_s", 2.0)),
        readiness_poll_interval_s=float(
            drone_raw.get("readiness_poll_interval_s", 0.25)
        ),
        takeoff_climb_m=float(drone_raw.get("takeoff_climb_m", 1.0)),
        camera_offset_m=drone_offset,
        camera_mount_rpy_deg=drone_mount,
    )

    motion = MotionLimits(
        max_speed_mps=float(motion_raw.get("max_speed_mps", 3.0)),
        max_acceleration_mps2=float(motion_raw.get("max_accel_mps2", 1.5)),
        max_yaw_rate_deg_s=float(motion_raw.get("max_yaw_rate_dps", 30.0)),
        max_yaw_acceleration_deg_s2=float(motion_raw.get("max_yaw_accel_dps2", 60.0)),
        control_hz=float(motion_raw.get("control_hz", 20.0)),
        min_duration_s=float(motion_raw.get("min_duration_s", 1.0)),
        passed_lateral_tolerance_m=float(motion_raw.get("passed_lateral_tolerance_m", 0.3)),
        **{
            # 回落值取 dataclass 默认值：默认值是实机验过的那一组，不在这里重写一份。
            name: float(motion_raw.get(name, getattr(MotionLimits, name)))
            for name in (
                "heading_hold_gain",
                "heading_rate_damping",
                "heading_deadband_deg",
                "heading_rate_deadband_dps",
            )
        },
    )
    depth_raw = _section(raw, "depth")
    depth = DepthSettings(
        mapping=str(depth_raw.get("mapping", "log_inverse_gray_v1")),
        min_m=float(depth_raw.get("min_m", 0.5)),
        max_m=float(depth_raw.get("max_m", 200.0)),
    )

    continuous_raw = motion_raw.get("continuous", {})
    if not isinstance(continuous_raw, dict):
        raise ValueError("motion.continuous must be a mapping")
    continuous_names = {item.name for item in fields(ContinuousTrackingSettings)}
    unknown = set(continuous_raw) - continuous_names
    if unknown:
        raise ValueError(
            f"unknown motion.continuous settings: {sorted(unknown)}"
        )
    continuous = ContinuousTrackingSettings(
        **{
            name: float(value)
            for name, value in continuous_raw.items()
        }
    )

    detection_raw = _section(raw, "target_detection")
    target_detection = TargetDetectionSettings(
        url=str(detection_raw.get("url", "")),
        model=str(detection_raw.get("model", "")),
        api_key=str(detection_raw.get("api_key", "")),
        api_key_env=str(detection_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(detection_raw.get("timeout_s", 30.0)),
        reasoning_effort=str(detection_raw.get("reasoning_effort", "none")),
        request_format=str(detection_raw.get("request_format", "openai")),
        max_completion_tokens=int(
            detection_raw.get("max_completion_tokens", 2048)
        ),
        max_image_side=int(detection_raw.get("max_image_side", 1024)),
        minimum_depth_m=float(detection_raw.get("minimum_depth_m", 0.5)),
    )

    planner_raw = _section(raw, "trajectory_planner")
    trajectory_planner = TrajectoryPlannerSettings(
        url=str(planner_raw.get("url", "")),
        model=str(planner_raw.get("model", "")),
        api_key=str(planner_raw.get("api_key", "")),
        api_key_env=str(planner_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(planner_raw.get("timeout_s", 30.0)),
        max_tokens=int(planner_raw.get("max_tokens", 256)),
        reasoning_effort=str(planner_raw.get("reasoning_effort", "none")),
        request_format=str(planner_raw.get("request_format", "openai")),
        point_count=int(planner_raw.get("point_count", 5)),
        max_step_m=float(planner_raw.get("max_step_m", 2.0)),
        max_yaw_step_deg=float(
            planner_raw.get("max_yaw_step_deg", 45.0)
        ),
    )
    if trajectory_planner.timeout_s <= 0:
        raise ValueError("trajectory_planner.timeout_s must be positive")
    if trajectory_planner.max_tokens <= 0:
        raise ValueError("trajectory_planner.max_tokens must be positive")

    server_raw = _section(app_raw, "server")
    bbox_raw = server_raw.get(
        "manual_bbox", [0.45, 0.82, 0.55, 0.92]
    )
    if not isinstance(bbox_raw, (list, tuple)) or len(bbox_raw) != 4:
        raise ValueError("server.manual_bbox must list four numbers")
    points_raw = server_raw.get("manual_surface_points", [])
    if not isinstance(points_raw, (list, tuple)):
        raise ValueError("server.manual_surface_points must be a list")
    points = []
    for point in points_raw:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(
                "server.manual_surface_points entries must be [u, v] pairs"
            )
        points.append((float(point[0]), float(point[1])))
    origins_raw = server_raw.get("allowed_origins", ["*"])
    if not isinstance(origins_raw, (list, tuple)):
        raise ValueError("server.allowed_origins must be a list")
    server = ServerSettings(
        host=str(server_raw.get("host", "127.0.0.1")),
        port=int(server_raw.get("port", 8000)),
        detector=str(server_raw.get("detector", "manual")),
        planner=str(server_raw.get("planner", "geometric")),
        manual_bbox=tuple(float(value) for value in bbox_raw),
        manual_surface_points=tuple(points),
        allowed_origins=tuple(str(origin) for origin in origins_raw),
        log_buffer_size=int(server_raw.get("log_buffer_size", 500)),
        reconnect_interval_s=float(server_raw.get("reconnect_interval_s", 2.0)),
        startup_timeout_s=float(server_raw.get("startup_timeout_s", 60.0)),
        video_fps=float(server_raw.get("video_fps", 10.0)),
        vehicles=tuple(
            str(vehicle) for vehicle in server_raw.get("vehicles", ("rover", "drone"))
        ),
        default_vehicle=str(server_raw.get("default_vehicle", "rover")),
    )
    vehicle_models = load_vehicle_models(
        _family_vehicle_models(families),
        maps=maps,
        vehicles=server.vehicles,
        families=families,
    )

    sliding_raw = _section(raw, "sliding_window")
    sliding_names = {item.name for item in fields(SlidingWindowSettings)}
    unknown = set(sliding_raw) - sliding_names
    if unknown:
        raise ValueError(f"unknown sliding_window settings: {sorted(unknown)}")
    sliding_window = SlidingWindowSettings(**sliding_raw)

    free_raw = _section(raw, "free_navigation")
    free_names = {item.name for item in fields(FreeNavigationSettings)}
    unknown = set(free_raw) - free_names
    if unknown:
        raise ValueError(f"unknown free_navigation settings: {sorted(unknown)}")
    free_navigation = FreeNavigationSettings(**free_raw)

    completion_raw = _section(raw, "task_completion")
    task_completion = TaskCompletionSettings(
        url=str(completion_raw.get("url", "")),
        model=str(completion_raw.get("model", "")),
        api_key=str(completion_raw.get("api_key", "")),
        api_key_env=str(completion_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(completion_raw.get("timeout_s", 20.0)),
        max_completion_tokens=int(
            completion_raw.get("max_completion_tokens", 2048)
        ),
        reasoning_effort=str(completion_raw.get("reasoning_effort", "none")),
        request_format=str(completion_raw.get("request_format", "openai")),
    )

    parser_raw = _section(raw, "task_parser")
    task_parser = TaskParserSettings(
        url=str(parser_raw.get("url", "")),
        model=str(parser_raw.get("model", "")),
        api_key=str(parser_raw.get("api_key", "")),
        api_key_env=str(parser_raw.get("api_key_env", "VLN_API_KEY")),
        timeout_s=float(parser_raw.get("timeout_s", 60.0)),
        max_completion_tokens=int(
            parser_raw.get("max_completion_tokens", 2048)
        ),
        reasoning_effort=str(parser_raw.get("reasoning_effort", "none")),
        request_format=str(parser_raw.get("request_format", "openai")),
    )

    visual_raw = _section(raw, "visual_goal_navigation")
    visual_names = {
        "enabled",
        "geometry_verified",
        "collision_check_enabled",
        "output_root",
        "limits",
        "completion_rules_file",
        "depth_foreground_filter",
    }
    unknown = set(visual_raw) - visual_names
    if unknown:
        raise ValueError(
            f"unknown visual-goal configuration: {sorted(unknown)}"
        )
    limits_raw = visual_raw.get("limits", {})
    if not isinstance(limits_raw, dict):
        raise ValueError("visual-goal limits must be a mapping")
    if "maximum_target_position_jump_m" in limits_raw:
        raise ValueError(
            "replace maximum_target_position_jump_m with "
            "target_position_update_radius_m (relative to first target anchor)"
        )
    if "target_center_margin" in limits_raw:
        raise ValueError(
            "remove target_center_margin: current target visibility is "
            "evaluated by completion model"
        )
    foreground_raw = visual_raw.get("depth_foreground_filter", {})
    if not isinstance(foreground_raw, dict):
        raise ValueError("depth_foreground_filter must be a mapping")
    flags = {}
    for name, default in (
        ("enabled", True),
        ("geometry_verified", False),
        ("collision_check_enabled", False),
    ):
        value = visual_raw.get(name, default)
        if type(value) is not bool:
            raise ValueError(f"{name} must be boolean")
        flags[name] = value
    completion_rules_file = visual_raw.get("completion_rules_file")
    if completion_rules_file is not None and (
        not isinstance(completion_rules_file, str)
        or not completion_rules_file.strip()
    ):
        raise ValueError("completion_rules_file must be non-empty text")
    visual_goal_navigation = VisualGoalNavigationSettings(
        **flags,
        output_root=str(visual_raw.get("output_root", "output/visual_goal")),
        limits=VisualGoalLimits(**limits_raw),
        completion_rules_file=completion_rules_file,
        completion_rules=(
            load_completion_rules(path, completion_rules_file)
            if flags["enabled"]
            else VisualGoalTriggerRules()
        ),
        depth_foreground_filter=DepthForegroundSettings(**foreground_raw),
    )

    return AppConfig(
        projectairsim=projectairsim,
        camera=camera,
        motion=motion,
        maps=maps,
        families=families,
        vehicle_models=vehicle_models,
        simulator=simulator,
        drone=drone,
        target_detection=target_detection,
        trajectory_planner=trajectory_planner,
        server=server,
        depth=depth,
        continuous=continuous,
        sliding_window=sliding_window,
        task_completion=task_completion,
        task_parser=task_parser,
        visual_goal_navigation=visual_goal_navigation,
        free_navigation=free_navigation,
        profile=_profile_info(raw, path),
        path=str(Path(path)),
    )


def load_completion_rules(
    config_path: str | Path, filename: str | None = None
) -> VisualGoalTriggerRules:
    """Load the reference visual-goal completion rules.

    With an explicit filename the path resolves inside the profile directory that
    owns ``config_path``, falling back to the deployment's ``config/`` when that
    directory has no copy. With no filename it reads the default profile's
    ``tasks/visual_goal_navigation.yaml``.
    """
    path = (
        shared_or_profile(config_path, filename)
        if filename is not None
        else default_config_path().parent
        / "tasks"
        / "visual_goal_navigation.yaml"
    )
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or set(raw) - {
        f.name for f in fields(VisualGoalTriggerRules)
    }:
        raise ValueError("invalid visual-goal completion rules")
    options = dict(raw)
    if "required_conditions" in options:
        if not isinstance(options["required_conditions"], list):
            raise ValueError("required_conditions must be a list")
        options["required_conditions"] = tuple(options["required_conditions"])
    return VisualGoalTriggerRules(**options)
