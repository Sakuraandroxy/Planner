"""The schema of ``config/simulators.yaml``: one block per simulator family.

A family says where its process comes from (driver, argv, stop argv), which ports it
owns, which interpreter can import its client, and what the scenes of its maps look
like. Parsing is strict: unknown keys are rejected, because a silently dropped
setting is the hardest class of simulator misconfiguration to find on the machine
that owns it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..domain.capabilities import _strings

#: There is one process driver. The key stays in the schema so that a config left
#: over from the launch-script era fails with a sentence instead of a stack trace.
KNOWN_DRIVERS = frozenset({"xpra_start"})


@dataclass(frozen=True)
class SimulatorFamily:
    """One simulator backend: how to start it, how to know it is up, what it offers."""

    id: str
    label: str
    #: There is one process driver: ``xpra_start``, which renders ``argv`` into an
    #: ``xpra start --start-child`` command. The key stays because a config left over
    #: from the launch-script era has to fail with a sentence, not a stack trace.
    driver: str = "xpra_start"
    #: Placeholder names resolved against the map entry plus this family.
    #: Every placeholder must be fillable, checked when the argv is rendered.
    argv: tuple[str, ...] = ()
    stop_argv: tuple[str, ...] = ()
    #: Readiness probes: ``window``, ``ports``, ``rpc``.
    probes: tuple[str, ...] = ("ports",)
    ports: dict[str, int] = field(default_factory=dict)
    #: Runtime entry point, module only. Never imported by the HTTP process.
    runtime: str = ""
    #: Absolute log directory template for the child simulator process.
    abslog_dir: str = "output/logs"
    #: The engine settings file, empty for families whose binary takes none
    #: (ProjectAirSim). It also carries the AirSim ownership fingerprint.
    settings_file: str = ""
    #: Connection data the runtime package needs to talk to the simulator. These
    #: live with the family because that is what "how to connect" means; the old
    #: branch kept the same four values in its own ``airsim`` config section.
    #: ``vehicle_name`` is not cosmetic: AirSim routes control calls by it, and a
    #: blank name silently fails against a build whose vehicle is named in
    #: settings.json (see planner/simulators/airsim/connection.py).
    host: str = ""
    vehicle_name: str = ""
    camera_id: str = ""
    timeout_s: float = 10.0
    #: How this family's scenes are captured (``camera`` in simulators.yaml).
    #: Resolution, field of view, mount offset and depth encoding belong to the
    #: simulator package, not to the navigation core; ``planner.config`` turns
    #: this mapping into ``CameraSettings`` because that type lives there.
    camera: dict[str, Any] = field(default_factory=dict)
    #: Appearance models this family can switch between (``vehicle_models``).
    #: Only a family that produces the meshes can offer them, so the list sits
    #: here instead of the program config; scene variants stay on each map.
    vehicle_models: dict[str, Any] = field(default_factory=dict)
    #: Scene conventions of this family (``scene_defaults``): actor names, camera
    #: ids, start view. Map entries repeat them only when a map deviates.
    scene_defaults: dict[str, Any] = field(default_factory=dict)
    #: Interpreter that can import this family's simulator client. Empty means the
    #: mission session runs on a worker thread inside the HTTP process, which is how
    #: ProjectAirSim has always worked. AirSim needs it because ``msgpack-python``
    #: (its transport) and ``msgpack`` (ProjectAirSim's) own the same module name, so
    #: no interpreter on this machine can hold both -- see docs/DEPLOYMENT.md 2.
    runtime_python: str = ""
    res_x: int = 1280
    res_y: int = 720
    #: argv for the rendered child process, joined by the driver into
    #: ``--start-child`` (the UE engines read the map name as argv[1]).
    child_argv: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        for name in ("id", "label"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"simulator family {name} must be non-empty text")
        if any(ch.isspace() for ch in self.id):
            raise ValueError("simulator family id must not contain whitespace")
        if self.driver not in KNOWN_DRIVERS:
            if self.driver == "script":
                raise ValueError(
                    f"simulator family '{self.id}' names driver 'script': the launch "
                    "scripts were removed, every family is started from the argv it "
                    "declares in config/simulators.yaml (see the airsim block for a "
                    "worked example)"
                )
            raise ValueError(
                f"simulator family '{self.id}' driver must be one of "
                f"{sorted(KNOWN_DRIVERS)}, got {self.driver!r}"
            )
        if not isinstance(self.ports, dict) or not self.ports:
            raise ValueError(f"simulator family '{self.id}' must declare at least one port")
        for name, value in self.ports.items():
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f"simulator family '{self.id}' port {name} must be in [1, 65535]")
        object.__setattr__(self, "probes", _strings(self.probes, name=f"simulator family '{self.id}' probes"))
        if not self.probes:
            raise ValueError(f"simulator family '{self.id}' must declare at least one probe")
        if not self.argv:
            raise ValueError(
                f"simulator family '{self.id}' needs an argv template for xpra"
            )
        if not self.stop_argv:
            raise ValueError(
                f"simulator family '{self.id}' needs a stop_argv template"
            )
        for name in ("res_x", "res_y"):
            value = getattr(self, name)
            if type(value) is not int or value < 64:
                raise ValueError(f"simulator family '{self.id}' {name} must be an integer of at least 64")


@dataclass(frozen=True)
class FamilyCatalog:
    """Every configured family, keyed by id."""

    families: tuple[SimulatorFamily, ...]
    default_family: str = "projectairsim"

    def __post_init__(self) -> None:
        ids = [family.id for family in self.families]
        if not self.families:
            raise ValueError("simulator catalog must contain at least one family")
        if len(set(ids)) != len(ids):
            raise ValueError("simulator family ids must be unique")
        if self.default_family not in ids:
            raise ValueError(
                f"simulator default_family {self.default_family!r} is not a configured family"
            )

    def entry(self, family_id: str) -> SimulatorFamily | None:
        for family in self.families:
            if family.id == family_id:
                return family
        return None

    def require(self, family_id: str) -> SimulatorFamily:
        family = self.entry(family_id)
        if family is None:
            raise ValueError(
                f"unknown simulator family {family_id!r}; expected one of {list(self.ids)}"
            )
        return family

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(family.id for family in self.families)

    @property
    def labels(self) -> dict[str, str]:
        return {family.id: family.label for family in self.families}


KNOWN_FAMILY_KEYS = frozenset(
    {
        "label", "driver", "argv", "child_argv", "stop_argv", "probes", "ports",
        "runtime", "abslog_dir", "settings_file", "res_x", "res_y",
        "host", "vehicle_name", "camera_id", "timeout_s", "runtime_python",
        "camera", "vehicle_models", "scene_defaults",
    }
)

#: Top-level sections of simulators.yaml that are not simulator families.
NON_FAMILY_SECTIONS = frozenset({"default_simulator", "default_family"})


def parse_family_catalog(raw: Any) -> FamilyCatalog:
    """Parse ``config/simulators.yaml``.

    Unknown keys are rejected rather than ignored: a silently dropped setting is
    the hardest class of simulator misconfiguration to find on the machine that
    owns it. A missing file is an error too, because the only replacement for it
    used to be a Python copy of the same argv template, and two copies of a command
    line is how a driver ends up launching a stack it cannot stop.
    """
    if not raw:
        raise ValueError(
            "simulator catalog is empty: config/simulators.yaml must define at "
            "least one family"
        )
    if not isinstance(raw, dict):
        raise ValueError("simulator catalog root must be a mapping")
    default_family = str(raw.get("default_simulator", raw.get("default_family", "")))
    families = []
    for family_id, item in raw.items():
        if family_id in NON_FAMILY_SECTIONS:
            continue
        if not isinstance(item, dict):
            raise ValueError(f"simulator family {family_id!r} must be a mapping")
        if "capabilities" in item:
            raise ValueError(
                f"simulator {family_id!r} must not declare capabilities: they are "
                "derived from the ports the simulator package implements "
                "(see planner/simulator/registry.py)"
            )
        unknown = set(item) - KNOWN_FAMILY_KEYS
        if unknown:
            raise ValueError(
                f"simulator family {family_id!r} has unknown settings {sorted(unknown)}; "
                f"expected any of {sorted(KNOWN_FAMILY_KEYS)}"
            )
        families.append(
            SimulatorFamily(
                id=str(family_id),
                label=str(item.get("label", family_id)),
                driver=str(item.get("driver", "xpra_start")),
                argv=tuple(str(part) for part in item.get("argv", ())),
                child_argv=tuple(str(part) for part in item.get("child_argv", ())),
                stop_argv=tuple(str(part) for part in item.get("stop_argv", ())),
                probes=tuple(item.get("probes", ("ports",))),
                ports={str(key): int(value) for key, value in (item.get("ports", {}) or {}).items()},
                runtime=str(item.get("runtime", "")),
                abslog_dir=str(item.get("abslog_dir", "output/logs")),
                settings_file=str(item.get("settings_file", "")),
                host=str(item.get("host", "")),
                vehicle_name=str(item.get("vehicle_name", "")),
                camera_id=str(item.get("camera_id", "")),
                timeout_s=float(item.get("timeout_s", 10.0)),
                runtime_python=str(item.get("runtime_python", "")),
                res_x=int(item.get("res_x", 1280)),
                res_y=int(item.get("res_y", 720)),
                camera=dict(item.get("camera") or {}),
                vehicle_models=dict(item.get("vehicle_models") or {}),
                scene_defaults=dict(item.get("scene_defaults") or {}),
            )
        )
    catalog = FamilyCatalog(
        families=tuple(families),
        default_family=default_family or (families[0].id if families else ""),
    )
    return catalog
