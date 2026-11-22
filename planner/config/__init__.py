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
from ..domain.view import VIEW_MODES
from ..domain.visual_goal_completion import VisualGoalTriggerRules
from ..domain.visual_goal_limits import VisualGoalLimits
from ..domain.motion import MotionLimits
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
class MapScene:
    """Scene identity of one map (``scene`` inside a ``maps.yaml`` entry).

    Everything here is read off the scene JSONC that the entry names: which file
    the server loads, how long the renderer needs, what the actors and cameras
    are called, and which camera the package starts on. It lives with the map
    because a different map means different answers; the navigation core does
    not see any of it.
    """

    #: Dual-vehicle scene JSONC inside ``config_path``.
    file: str = ""
    #: World scene-config directory, relative to the repository root.
    config_path: str = ""
    #: The dual scene starts with its clock paused; this only waits for the renderer.
    delay_after_load_s: float = 2.0
    #: Actor names, which must match the scene JSONC.
    rover_name: str = "Rover1"
    drone_name: str = "Drone1"
    #: Sensor ids, which must match the robot JSONC files.
    front_camera: str = "FrontCamera"
    chase_camera: str = "Chase"
    down_camera: str = "DownCamera"
    #: Streaming views whose camera sets ``capture-enabled: false``.
    uncapturable_views: tuple[str, ...] = ()
    #: Which camera the engine shows right after this scene loads.
    view_cycle_start: str = "drone_front"
    #: Scene variant per appearance model on this map (``hexarotor: file.jsonc``).
    #: Models without an entry use ``file``.
    vehicle_scenes: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        if bool(self.file.strip()) != bool(self.config_path.strip()):
            raise ValueError(
                "map scene declares half a pair: file and config_path are "
                "required together"
            )
        for name in (
            "rover_name",
            "drone_name",
            "front_camera",
            "chase_camera",
            "down_camera",
        ):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"map scene {name} must be non-empty")
        views = tuple(self.uncapturable_views)
        if len(set(views)) != len(views):
            raise ValueError("map scene uncapturable_views must not repeat")
        unknown = [view for view in views if view not in VIEW_MODES]
        if unknown:
            raise ValueError(
                "map scene uncapturable_views has unknown views: "
                + ", ".join(unknown)
            )
        if self.view_cycle_start not in VIEW_MODES:
            raise ValueError(
                f"map scene view_cycle_start must be one of "
                f"{', '.join(VIEW_MODES)}, got {self.view_cycle_start!r}"
            )
        for model_id, scene_file in self.vehicle_scenes:
            if not model_id.strip() or not scene_file.strip():
                raise ValueError(
                    "map scene vehicle_scenes needs a model id and a scene file"
                )

    def scene_for(self, model_id: str | None) -> str:
        for key, value in self.vehicle_scenes:
            if key == model_id:
                return value
        return self.file


@dataclass(frozen=True)
class MapEntry:
    """One switchable simulator map (``config/maps.yaml``).

    A map belongs to one simulator family. Scene-driven families need
    ``launch_map``, ``scene_file`` and ``sim_config_path``; families started
    straight from an engine binary (AirSim for UE4) need ``binary`` and
    ``map_name`` instead. Which pair applies follows from what the entry
    declares, not from a family-name test, so a new family needs no change here.
    """

    id: str
    label: str
    # Kept for scene selection and older configs; the engine takes ``map_name``.
    # Defaults to ``map_name`` when empty, so neither has to be written twice.
    launch_map: str = ""
    # Scene identity: which JSONC to load and what its actors are called.
    scene: MapScene = field(default_factory=MapScene)
    # Simulator package that drives this map, resolved by
    # ``planner.simulator.registry``. Named after the software, not the engine
    # version, and required: no map inherits a simulator by default.
    simulator: str = ""
    # Engine binary for families launched straight from an executable.
    binary: str = ""
    # Map name passed to that binary (AirSim takes it as argv[1]).
    map_name: str = ""

    def __post_init__(self) -> None:
        for name in ("id", "label", "simulator"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"map {name} must be non-empty text")
        for name in ("id", "simulator"):
            value = getattr(self, name)
            if any(ch.isspace() for ch in value):
                raise ValueError(f"map {name} must not contain whitespace")
        scene_pair = bool(self.scene_file.strip()) == bool(self.sim_config_path.strip())
        if not scene_pair:
            raise ValueError(
                f"map '{self.id}' declares half a scene pair: scene.file and "
                "scene.config_path are required together"
            )
        binary_pair = bool(self.binary.strip()) == bool(self.map_name.strip())
        if not binary_pair:
            raise ValueError(
                f"map '{self.id}' declares half a binary pair: binary and "
                "map_name are required together"
            )
        if not self.scene_file.strip() and not self.binary.strip():
            # Allowed on the fallback entry a catalog-less config produces; the map
            # catalog itself rejects such an entry right after parsing.
            pass
        if not self.launch_map.strip():
            # Binary families name their launch target through map_name.
            object.__setattr__(self, "launch_map", self.map_name)

    @property
    def scene_file(self) -> str:
        """Scene JSONC this map loads; empty for maps without a scene."""
        return self.scene.file

    @property
    def sim_config_path(self) -> str:
        """Scene-config directory this map loads from."""
        return self.scene.config_path

    @property
    def uses_scene_config(self) -> bool:
        return bool(self.scene_file.strip() and self.sim_config_path.strip())


@dataclass(frozen=True)
class VehicleModelOption:
    """One appearance model the console may select (``vehicle_models``)."""

    id: str
    label: str

    def __post_init__(self) -> None:
        for name in ("id", "label"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"vehicle model {name} must be non-empty text")
            if name == "id" and any(ch.isspace() for ch in value):
                raise ValueError("vehicle model id must not contain whitespace")


@dataclass(frozen=True)
class VehicleModelGroup:
    """Selectable appearance models for one vehicle id."""

    vehicle: str
    default: str
    options: tuple[VehicleModelOption, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.vehicle, str) or not self.vehicle.strip():
            raise ValueError("vehicle model group must name a vehicle")
        if not self.options:
            raise ValueError(
                f"vehicle model group '{self.vehicle}' needs at least one option"
            )
        ids = [option.id for option in self.options]
        if len(set(ids)) != len(ids):
            raise ValueError(
                f"vehicle model ids for '{self.vehicle}' must be unique"
            )
        if self.default not in ids:
            raise ValueError(
                f"default model '{self.default}' is not an option for "
                f"'{self.vehicle}'"
            )

    def option(self, model_id: str) -> VehicleModelOption | None:
        for option in self.options:
            if option.id == model_id:
                return option
        return None

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(option.id for option in self.options)

    @property
    def labels(self) -> dict[str, str]:
        return {option.id: option.label for option in self.options}


@dataclass(frozen=True)
class VehicleModelCatalog:
    """Appearance models per vehicle; empty ``groups`` disables switching."""

    groups: tuple[VehicleModelGroup, ...] = ()

    def __post_init__(self) -> None:
        vehicles = [group.vehicle for group in self.groups]
        if len(set(vehicles)) != len(vehicles):
            raise ValueError("vehicle model groups must be unique per vehicle")
        # Scene variants live on the map entry, so two vehicles can no longer
        # point one map at two different scenes; nothing to reconcile here.

    def group(self, vehicle: str) -> VehicleModelGroup | None:
        for group in self.groups:
            if group.vehicle == vehicle:
                return group
        return None

    @property
    def vehicles(self) -> tuple[str, ...]:
        return tuple(group.vehicle for group in self.groups)

    @property
    def defaults(self) -> dict[str, str]:
        return {group.vehicle: group.default for group in self.groups}


def default_vehicle_models() -> VehicleModelCatalog:
    """Deployments without a ``vehicle_models`` section keep the default look."""
    return VehicleModelCatalog(
        groups=(
            VehicleModelGroup(
                "rover",
                "offroad",
                (VehicleModelOption("offroad", "越野车"),),
            ),
            VehicleModelGroup(
                "drone",
                "quadrotor",
                (VehicleModelOption("quadrotor", "四旋翼"),),
            ),
        )
    )


@dataclass(frozen=True)
class MapCatalog:
    """Map ids the console may switch to; ``default_map`` must be listed."""

    default_map: str
    maps: tuple[MapEntry, ...]

    def __post_init__(self) -> None:
        if not self.maps:
            raise ValueError("maps catalog must contain at least one map")
        ids = [entry.id for entry in self.maps]
        if len(set(ids)) != len(ids):
            raise ValueError("map ids must be unique")
        if self.default_map not in ids:
            raise ValueError("default_map must appear in maps")

    def simulator_ids(self) -> tuple[str, ...]:
        """Every simulator referenced by a map, in catalog order."""
        return tuple(dict.fromkeys(entry.simulator for entry in self.maps))

    def entry(self, map_id: str) -> MapEntry | None:
        for entry in self.maps:
            if entry.id == map_id:
                return entry
        return None

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(entry.id for entry in self.maps)

    @property
    def labels(self) -> dict[str, str]:
        return {entry.id: entry.label for entry in self.maps}


@dataclass(frozen=True)
class ProfileInfo:
    """How one profile is introduced: the console shows ``label``, and ``id`` is the
    directory name and the value the switch endpoint takes.
    """

    id: str = ""
    label: str = ""
    note: str = ""
    #: The trajectory planner this profile calls, shown so a viewer can see what
    #: the switch actually changed. Read from the raw section, not validated here.
    model: str = ""


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


def _profile_info(raw: dict[str, Any], path: str | Path) -> ProfileInfo:
    section = _section(raw, "profile")
    name = profile_of(path)
    declared = str(section.get("id", "")).strip()
    if declared and name and declared != name:
        raise ValueError(
            f"profile id '{declared}' does not match its directory '{name}': "
            f"rename it or move the file, the directory is what the switch uses"
        )
    identifier = name or declared
    planner = raw.get("trajectory_planner")
    model = str(planner.get("model", "")).strip() if isinstance(planner, dict) else ""
    return ProfileInfo(
        id=identifier,
        label=str(section.get("label", "")).strip() or identifier,
        note=str(section.get("note", "")).strip(),
        model=model,
    )


#: Sections a profile is allowed to differ in. Everything else describes the
#: deployment, not the navigation: the simulator process, the engine's own camera
#: mount, the appearance models and the HTTP service are not rebuilt when a
#: profile switches, so a difference there is either inert or a quiet
#: disagreement with an engine that is already running.
PROFILE_SWITCHABLE_SECTIONS = frozenset({
    "profile",
    "drone",
    "depth",
    "motion",
    "sliding_window",
    "target_detection",
    "trajectory_planner",
    "task_completion",
    "task_parser",
    "visual_goal_navigation",
    "free_navigation",
})


def _profile_raw(profile: str) -> dict[str, Any]:
    raw = yaml.safe_load(
        profile_config_path(profile).read_text(encoding="utf-8")
    ) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"profile '{profile}' config root must be a mapping")
    return raw


def profile_drift(reference: str | None = None) -> tuple[str, ...]:
    """Where a profile differs from the reference outside ``PROFILE_SWITCHABLE_SECTIONS``.

    Empty means every profile only changes what switching can actually apply.
    ``scripts/check_profiles.py`` prints this and fails a deployment on it.
    """
    profiles = available_profiles()
    base = reference or default_profile()
    if base not in profiles:
        return (f"reference profile '{base}' is not installed",)
    reference_raw = _profile_raw(base)
    drift = []
    for name in profiles:
        if name == base:
            continue
        raw = _profile_raw(name)
        for section in sorted(
            (set(reference_raw) | set(raw)) - PROFILE_SWITCHABLE_SECTIONS
        ):
            if reference_raw.get(section) != raw.get(section):
                drift.append(
                    f"{name}: section '{section}' differs from '{base}'"
                )
    return tuple(drift)


def profile_catalog() -> tuple[ProfileInfo, ...]:
    """Every installed profile with the name it introduces itself by.

    Reads only the ``profile`` section of each base file, so a page can list the
    profiles without building simulators or validating the whole configuration.
    """
    entries = []
    for name in available_profiles():
        raw = _profile_raw(name)
        entries.append(_profile_info(raw, profile_config_path(name)))
    return tuple(entries)


def load_map_catalog(
    path: str | Path,
    *,
    fallback: MapEntry | None = None,
    default_simulator: str = "projectairsim",
    scene_defaults: dict[str, dict[str, Any]] | None = None,
) -> MapCatalog:
    """Load ``maps.yaml``; without the file fall back to a single map entry.

    Tests and small deployments use a config file with no catalog next to it;
    they reconnect to whatever scene ``projectairsim`` names instead of
    switching maps. Explicit catalog entries are checked against the
    repository tree so a typo cannot silently select a missing scene.
    """
    catalog_path = Path(path)
    if not catalog_path.is_file():
        if fallback is None:
            raise FileNotFoundError(f"map catalog not found: {catalog_path}")
        return MapCatalog(default_map=fallback.id, maps=(fallback,))
    raw = yaml.safe_load(catalog_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError("map catalog root must be a mapping")
    entries_raw = raw.get("maps")
    if not isinstance(entries_raw, list) or not entries_raw:
        raise ValueError("map catalog must list at least one map")
    binary_root = _binary_root(raw.get("root"), catalog_path)
    entries = []
    for item in entries_raw:
        if not isinstance(item, dict):
            raise ValueError("map catalog entries must be mappings")
        entries.append(
            MapEntry(
                id=str(item.get("id", "")),
                label=str(item.get("label", "")),
                simulator=str(item.get("simulator", default_simulator)),
                launch_map=str(item.get("launch_map", "")),
                scene=_parse_map_scene(
                    item.get("scene"),
                    map_id=str(item.get("id", "")),
                    defaults=(scene_defaults or {}).get(
                        str(item.get("simulator", default_simulator)), {}
                    ),
                ),
                binary=_resolve_binary(
                    str(item.get("binary", "")),
                    binary_root,
                    map_id=str(item.get("id", "")),
                ),
                map_name=str(item.get("map_name", "")),
            )
        )
    catalog = MapCatalog(
        default_map=str(raw.get("default_map", "")),
        maps=tuple(entries),
    )
    root = _project_root()
    for entry in catalog.maps:
        if not entry.scene_file.strip() and not entry.binary.strip():
            raise ValueError(
                f"map '{entry.id}' must declare either scene.file plus "
                "scene.config_path or binary plus map_name"
            )
        # Only scene-driven maps carry repository files worth pre-validating;
        # binary-launched maps are checked on the machine that owns them.
        if not entry.uses_scene_config:
            continue
        config_dir = Path(entry.sim_config_path)
        if not config_dir.is_absolute():
            config_dir = root / config_dir
        if not config_dir.is_dir():
            raise ValueError(
                f"map '{entry.id}' sim_config_path is not a directory: "
                f"{entry.sim_config_path}"
            )
        if not (config_dir / entry.scene_file).is_file():
            raise ValueError(
                f"map '{entry.id}' scene_file is missing: "
                f"{entry.sim_config_path}/{entry.scene_file}"
            )
        for model_id, scene_file in entry.scene.vehicle_scenes:
            if not (config_dir / scene_file).is_file():
                raise ValueError(
                    f"map '{entry.id}' scene variant for model '{model_id}' "
                    f"is missing: {entry.sim_config_path}/{scene_file}"
                )
    return catalog


def _binary_root(raw: Any, catalog_path: Path) -> Path | None:
    """Package root the map binaries hang off (``root`` in maps.yaml).

    ``/roxy/vln`` on this machine. Everything under it is written relative to
    that one line, so moving the simulator packages to another machine does not
    mean editing ten entries.
    """
    value = str(raw or "").strip()
    if not value:
        return None
    root = Path(value)
    if root.is_absolute():
        return root
    # Relative roots resolve against the catalog's directory, like sim_config_path.
    return (catalog_path.parent / root).resolve()


def _resolve_binary(value: str, root: Path | None, *, map_id: str) -> str:
    """Engine binary for one map, joined to the package root.

    An absolute path is kept as written: a catalog may pin one entry to a build
    that lives outside the root.
    """
    if not value:
        return ""
    path = Path(value)
    if path.is_absolute():
        return str(path)
    if root is None:
        raise ValueError(
            f"map '{map_id}' binary is relative but maps.yaml declares no root"
        )
    return str(root / path)


def _parse_map_scene(
    raw: Any,
    *,
    map_id: str,
    defaults: dict[str, Any] | None = None,
) -> MapScene:
    """Parse the ``scene`` block of one ``maps.yaml`` entry.

    ``defaults`` are the scene conventions of the map's simulator (``scene_defaults``
    in simulators.yaml): names, cameras and start view an entry only repeats when
    it deviates. Strict on unknown keys, because a misspelled scene setting would
    otherwise load the map with the wrong actor names.
    """
    if raw is None and not defaults:
        return MapScene()
    if raw is not None and not isinstance(raw, dict):
        raise ValueError(f"map '{map_id}' scene must be a mapping")
    merged = dict(defaults or {})
    merged.update(raw or {})
    raw = merged
    known = {
        "file",
        "config_path",
        "delay_after_load_s",
        "rover_name",
        "drone_name",
        "front_camera",
        "chase_camera",
        "down_camera",
        "uncapturable_views",
        "view_cycle_start",
        "vehicle_scenes",
    }
    unknown = set(raw) - known
    if unknown:
        raise ValueError(
            f"map '{map_id}' scene has unknown settings {sorted(unknown)}; "
            f"expected any of {sorted(known)}"
        )
    vehicles_raw = raw.get("vehicle_scenes") or {}
    if not isinstance(vehicles_raw, dict):
        raise ValueError(f"map '{map_id}' scene.vehicle_scenes must be a mapping")
    return MapScene(
        file=str(raw.get("file", "")),
        config_path=str(raw.get("config_path", "")),
        delay_after_load_s=float(raw.get("delay_after_load_s", 2.0)),
        rover_name=str(raw.get("rover_name", "Rover1")),
        drone_name=str(raw.get("drone_name", "Drone1")),
        front_camera=str(raw.get("front_camera", "FrontCamera")),
        chase_camera=str(raw.get("chase_camera", "Chase")),
        down_camera=str(raw.get("down_camera", "DownCamera")),
        uncapturable_views=tuple(
            str(view) for view in raw.get("uncapturable_views", ())
        ),
        view_cycle_start=str(raw.get("view_cycle_start", "drone_front")),
        vehicle_scenes=tuple(
            (str(model_id).strip(), str(scene).strip())
            for model_id, scene in vehicles_raw.items()
        ),
    )


def _flatten_scene_defaults(
    raw: dict[str, Any], *, simulator: str
) -> dict[str, Any]:
    """Turn a simulator's ``scene_defaults`` into keys ``MapScene`` understands.

    The three camera ids are written as one ``cameras`` mapping, because an entry
    overrides them together or not at all.
    """
    if not raw:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(
            f"simulator '{simulator}' scene_defaults must be a mapping"
        )
    out = dict(raw)
    cameras = out.pop("cameras", None) or {}
    if not isinstance(cameras, dict):
        raise ValueError(
            f"simulator '{simulator}' scene_defaults.cameras must be a mapping"
        )
    unknown_cameras = set(cameras) - {"front", "chase", "down"}
    if unknown_cameras:
        raise ValueError(
            f"simulator '{simulator}' scene_defaults.cameras has unknown keys "
            f"{sorted(unknown_cameras)}; expected front, chase, down"
        )
    for slot, value in cameras.items():
        out[f"{slot}_camera"] = value
    return out


def load_vehicle_models(
    raw: dict[str, Any],
    *,
    maps: MapCatalog,
    vehicles: tuple[str, ...],
    families: FamilyCatalog,
) -> VehicleModelCatalog:
    """Parse the ``vehicle_models`` section and check its scene variants.

    Coverage is required for every map whose family advertises the ``models``
    action; appearance models are switched through the UE5 scene variants, so
    binary-launched families (AirSim UE4) are out of scope and must not be
    named here. The section is optional; without it the default appearance
    stays selected.
    """
    if not raw:
        return default_vehicle_models()
    groups = []
    for vehicle, group_raw in raw.items():
        vehicle_id = str(vehicle).strip()
        if vehicle_id not in vehicles:
            raise ValueError(
                f"vehicle_models names unknown vehicle '{vehicle}', "
                f"expected one of {list(vehicles)}"
            )
        if not isinstance(group_raw, dict):
            raise ValueError(
                f"vehicle_models.{vehicle_id} must be a mapping"
            )
        options_raw = group_raw.get("options")
        if not isinstance(options_raw, list) or not options_raw:
            raise ValueError(
                f"vehicle_models.{vehicle_id}.options must be a non-empty list"
            )
        options = []
        for item in options_raw:
            if not isinstance(item, dict):
                raise ValueError(
                    f"vehicle_models.{vehicle_id}.options entries must be mappings"
                )
            unknown = set(item) - {"id", "label"}
            if unknown:
                raise ValueError(
                    f"vehicle_models.{vehicle_id}.options has unknown settings "
                    f"{sorted(unknown)}; a scene variant is declared by the map "
                    "entry in maps.yaml (scene.vehicle_scenes), not here"
                )
            options.append(
                VehicleModelOption(
                    id=str(item.get("id", "")),
                    label=str(item.get("label", "")),
                )
            )
        groups.append(
            VehicleModelGroup(
                vehicle_id,
                str(group_raw.get("default", "")),
                tuple(options),
            )
        )
    catalog = VehicleModelCatalog(groups=tuple(groups))
    from ..simulator import registry

    modelable = {
        entry.id
        for entry in maps.maps
        if registry.capabilities_for(entry.simulator).supports("models")
    }
    variant_options = {
        option.id
        for group in catalog.groups
        for option in group.options
        if option.id != group.default
    }
    for map_id in modelable:
        entry = maps.entry(map_id)
        if entry is None:
            raise ValueError(
                f"map catalog lists simulator '{map_id}' but no such entry exists"
            )
        declared = {model_id for model_id, _ in entry.scene.vehicle_scenes}
        extra = declared - variant_options
        if extra:
            raise ValueError(
                f"map '{map_id}' declares scene variants for models that are "
                f"not non-default appearance options: {sorted(extra)}"
            )
        # A map without a variant for a model keeps its own scene, which is what
        # the model list promises: the look changes only where a scene exists.

    return catalog


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


def _family_vehicle_models(families: FamilyCatalog) -> dict[str, Any]:
    """Appearance options declared by a simulator family.

    Only the family that produces the meshes can offer a different look, so the
    list sits in simulators.yaml; at most one family may declare it. Scene
    variants per map stay in maps.yaml.
    """
    declaring = [family for family in families.families if family.vehicle_models]
    if len(declaring) > 1:
        raise ValueError(
            "only one simulator family may declare vehicle_models, got "
            + ", ".join(sorted(family.id for family in declaring))
        )
    return dict(declaring[0].vehicle_models) if declaring else {}


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