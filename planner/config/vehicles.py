"""Vehicle appearance catalog: which looks the console may select."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .engine_family import FamilyCatalog
from .maps import MapCatalog


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
