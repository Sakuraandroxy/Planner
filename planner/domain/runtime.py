"""Declarative simulator families and the capability flags they publish.

Adding a simulator means adding one block to ``config/simulators.yaml``. No
module in the HTTP, controller or console layer may branch on a family name;
they read the flags below instead. ``actions`` is the single gate for the
optional controls (home, appearance models, trajectory, video), and
``task_kinds`` is what the task parser is allowed to produce for a map.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

#: Controls the console may render for a map. Anything absent returns 501.
KNOWN_ACTIONS = frozenset({"home", "models", "trajectory", "video", "weather"})
KNOWN_DRIVERS = frozenset({"script", "xpra_start"})


def _strings(value: Any, *, name: str) -> tuple[str, ...]:
    if isinstance(value, str) or not isinstance(value, (list, tuple)):
        raise ValueError(f"{name} must be a list of strings")
    items = tuple(str(item).strip() for item in value)
    if any(not item for item in items):
        raise ValueError(f"{name} entries must be non-empty text")
    if len(set(items)) != len(items):
        raise ValueError(f"{name} entries must be unique")
    return items


@dataclass(frozen=True)
class Capabilities:
    """What the console and the API may offer for one simulator family."""

    vehicles: tuple[str, ...] = ("drone",)
    view_modes: tuple[str, ...] = ()
    actions: tuple[str, ...] = ()
    task_kinds: tuple[str, ...] = ()
    # Canonical weather field -> name this simulator's RPC expects.
    weather_fields: dict[str, str] = field(default_factory=dict)
    weather_presets: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "vehicles", _strings(self.vehicles, name="capabilities.vehicles"))
        object.__setattr__(self, "view_modes", _strings(self.view_modes, name="capabilities.view_modes"))
        object.__setattr__(self, "actions", _strings(self.actions, name="capabilities.actions"))
        object.__setattr__(self, "task_kinds", _strings(self.task_kinds, name="capabilities.task_kinds"))
        unknown = set(self.actions) - KNOWN_ACTIONS
        if unknown:
            raise ValueError(
                f"capabilities.actions has unknown entries {sorted(unknown)}; "
                f"expected any of {sorted(KNOWN_ACTIONS)}"
            )
        if not self.vehicles:
            raise ValueError("capabilities.vehicles must list at least one vehicle")
        if not self.task_kinds:
            raise ValueError("capabilities.task_kinds must list at least one task kind")
        if not isinstance(self.weather_fields, dict) or any(
            not str(key).strip() or not str(value).strip()
            for key, value in self.weather_fields.items()
        ):
            raise ValueError("capabilities.weather_fields must map names to non-empty text")
        object.__setattr__(
            self, "weather_presets", _strings(self.weather_presets, name="capabilities.weather_presets")
        )

    @property
    def supports_views(self) -> bool:
        return bool(self.view_modes)

    def supports(self, action: str) -> bool:
        return action in self.actions

    def as_dict(self) -> dict[str, Any]:
        return {
            "vehicles": list(self.vehicles),
            "view_modes": list(self.view_modes),
            "actions": list(self.actions),
            "task_kinds": list(self.task_kinds),
            "weather_fields": dict(self.weather_fields),
            "weather_presets": list(self.weather_presets),
        }


@dataclass(frozen=True)
class SimulatorFamily:
    """One simulator backend: how to start it, how to know it is up, what it offers."""

    id: str
    label: str
    #: ``script`` runs launch/stop scripts (the UE5 stack in this repository);
    #: ``xpra_start`` builds an ``xpra start --start-child`` argv from ``argv``.
    driver: str = "script"
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
    #: Only used by driver ``script``: the launch and stop script paths.
    launch_script: str = ""
    stop_script: str = ""
    #: Only used by driver ``xpra_start``: the engine settings file and the
    #: window size handed to the binary as ``-ResX`` / ``-ResY``.
    settings_file: str = ""
    res_x: int = 1280
    res_y: int = 720
    #: argv for the rendered child process, joined by the driver into
    #: ``--start-child`` (UE4 AirSim reads the map name as argv[1]).
    child_argv: tuple[str, ...] = ()


    def __post_init__(self) -> None:
        for name in ("id", "label"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"simulator family {name} must be non-empty text")
        if any(ch.isspace() for ch in self.id):
            raise ValueError("simulator family id must not contain whitespace")
        if self.driver not in KNOWN_DRIVERS:
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
        if self.driver == "xpra_start" and not self.argv:
            raise ValueError(
                f"simulator family '{self.id}' uses driver 'xpra_start' and needs an argv template"
            )
        if self.driver == "xpra_start" and not self.stop_argv:
            raise ValueError(
                f"simulator family '{self.id}' uses driver 'xpra_start' and needs a stop_argv template"
            )
        if self.driver == "script" and not (self.launch_script and self.stop_script):
            raise ValueError(
                f"simulator family '{self.id}' uses driver 'script' and needs launch_script and stop_script"
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


def projectairsim_family(simulator) -> SimulatorFamily:
    """Build the ProjectAirSim entry from the existing ``simulator`` section.

    A deployment without ``config/simulators.yaml`` keeps today's behaviour:
    the launch and stop scripts drive the stack. Capabilities are never read
    from here; the registry derives them from the ports the package implements.
    """
    return SimulatorFamily(
        id="projectairsim",
        label="ProjectAirSim (UE5)",
        driver="script",
        launch_script=simulator.launch_script,
        stop_script=simulator.stop_script,
        ports={"xpra": simulator.xpra_port, "topics": simulator.topics_port, "services": simulator.services_port},
        runtime="planner.runtimes.ue5",
    )


KNOWN_FAMILY_KEYS = frozenset(
    {
        "label", "driver", "argv", "child_argv", "stop_argv", "probes", "ports",
        "runtime", "abslog_dir", "launch_script", "stop_script", "settings_file",
        "res_x", "res_y",
    }
)


def parse_family_catalog(raw: Any, *, fallback: SimulatorFamily | None = None) -> FamilyCatalog:
    """Parse ``config/simulators.yaml``; an empty document keeps the fallback.

    Unknown keys are rejected rather than ignored: a silently dropped setting is
    the hardest class of simulator misconfiguration to find on the machine that
    owns it.
    """
    if not raw:
        if fallback is None:
            raise ValueError("simulator catalog is empty")
        return FamilyCatalog(families=(fallback,), default_family=fallback.id)
    if not isinstance(raw, dict):
        raise ValueError("simulator catalog root must be a mapping")
    default_family = str(raw.get("default_simulator", raw.get("default_family", "")))
    families = []
    for family_id, item in raw.items():
        if family_id in {"default_simulator", "default_family"}:
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
                driver=str(item.get("driver", "script")),
                argv=tuple(str(part) for part in item.get("argv", ())),
                child_argv=tuple(str(part) for part in item.get("child_argv", ())),
                stop_argv=tuple(str(part) for part in item.get("stop_argv", ())),
                probes=tuple(item.get("probes", ("ports",))),
                ports={str(key): int(value) for key, value in (item.get("ports", {}) or {}).items()},
                runtime=str(item.get("runtime", "")),
                abslog_dir=str(item.get("abslog_dir", "output/logs")),
                launch_script=str(item.get("launch_script", "")),
                stop_script=str(item.get("stop_script", "")),
                settings_file=str(item.get("settings_file", "")),
                res_x=int(item.get("res_x", 1280)),
                res_y=int(item.get("res_y", 720)),

            )
        )
    catalog = FamilyCatalog(
        families=tuple(families),
        default_family=default_family or (families[0].id if families else ""),
    )
    if fallback is not None and catalog.entry(fallback.id) is None:
        # The file exists but forgets the family the rest of the config assumes.
        return FamilyCatalog(
            families=(fallback, *catalog.families), default_family=catalog.default_family
        )
    return catalog
