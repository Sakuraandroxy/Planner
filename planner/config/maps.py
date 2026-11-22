"""Map catalog: which simulator maps exist and what scene each one loads."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from ..domain.view import VIEW_MODES
from .paths import _project_root


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
