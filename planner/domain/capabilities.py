"""What the console and the API may offer for one simulator family.

Derived in ``planner.assembly.requirements`` from the ports a simulator
implements, then written to the console by ``planner.adapters.http``. The action
vocabulary is validated at construction so a typo cannot silently hide a control.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

#: Controls the console may render for a map. Anything absent returns 501.
KNOWN_ACTIONS = frozenset({"home", "models", "trajectory", "video", "weather"})


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
