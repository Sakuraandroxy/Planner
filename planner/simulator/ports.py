"""The bundle a simulator hands the shared layer, and what it implies.

``SimulatorPorts`` is the only seam between the task logic and a simulator. The
shared layer asks for a port and either gets an object or gets ``None``; it
never asks which simulator is running.

``capabilities()`` derives the console's feature flags from those slots. That is
deliberate: a config file that lists capabilities can drift from what the code
implements, and the drift only shows up as a broken button on a machine with a
simulator attached.

The bundle answers exactly one question: *can this engine perform this action?*
Anything the service or the shared planner provides on its own -- trajectory
telemetry, logs, the task list, the instruction card -- must not be modelled
here, because then a generic feature becomes a per-simulator permission and a
simulator that never implemented the flag silently loses functionality it was
never missing. ``trajectory`` used to be gated this way: the port held the
service's own hub, the endpoint read that hub directly, and nothing consulted
the port except the gate.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

from planner.domain import Capabilities
from planner.ports import (
    AppearanceModelControl,
    FramePublisher,
    ReturnHome,
    StreamingViews,
    WeatherControl,
)
from planner.ports import (
    ControlledObservationSource,
    MovingObservationSource,
    ObservationSource,
)
from planner.ports import StreamingVehicle
from planner.ports import Vehicle

#: Task kind -> port slots it cannot run without. Keys are ``SimulatorPorts``
#: field names, so adding a task means adding one row, not one branch.
TASK_REQUIREMENTS: dict[str, tuple[str, ...]] = {
    "basic_motion": ("vehicle",),
    "visual_goal_navigation": ("vehicle", "observation"),
    "free_navigation": ("vehicle", "observation", "streaming"),
}

#: Port slot -> the ``actions`` flag it unlocks. Only engine-dependent controls
#: belong here; see the module docstring for why generic features stay out.
ACTION_PORTS: dict[str, str] = {
    "home": "home",
    "models": "models",
    "video": "frames",
    "weather": "weather",
}


@dataclass(frozen=True)
class SimulatorPorts:
    """Everything a simulator implements. Required slots cannot be absent."""

    vehicle: Vehicle
    observation: ObservationSource
    streaming: StreamingVehicle | None = None
    controlled_capture: ControlledObservationSource | None = None
    moving_capture: MovingObservationSource | None = None
    weather: WeatherControl | None = None
    views: StreamingViews | None = None
    models: AppearanceModelControl | None = None
    frames: FramePublisher | None = None
    home: ReturnHome | None = None
    vehicles: tuple[str, ...] = ("drone",)
    #: Task kinds the shared layer may offer even when every port is present.
    #: A simulator that has not been flown with a task lists it out of here.
    enabled_tasks: tuple[str, ...] = ()

    def slot(self, name: str) -> object | None:
        return getattr(self, name, None)

    def missing(self, required: tuple[str, ...]) -> tuple[str, ...]:
        return tuple(name for name in required if self.slot(name) is None)

    def available_tasks(self) -> tuple[str, ...]:
        return tuple(
            kind
            for kind, required in TASK_REQUIREMENTS.items()
            if not self.missing(required)
            and (not self.enabled_tasks or kind in self.enabled_tasks)
        )

    def capabilities(self) -> Capabilities:
        actions = tuple(
            action
            for action, slot in ACTION_PORTS.items()
            if self.slot(slot) is not None
        )
        view_modes: tuple[str, ...] = ()
        if self.views is not None:
            view_modes = tuple(self.views.view_modes())
        weather_fields: dict[str, str] = {}
        weather_presets: tuple[str, ...] = ()
        if self.weather is not None:
            weather_fields = dict(self.weather.fields())
            weather_presets = tuple(self.weather.presets())
        return Capabilities(
            vehicles=tuple(self.vehicles),
            view_modes=view_modes,
            actions=actions,
            task_kinds=self.available_tasks(),
            weather_fields=weather_fields,
            weather_presets=weather_presets,
        )

    def implemented(self) -> tuple[str, ...]:
        """Port slot names holding an object, for logs and architecture tests."""
        return tuple(
            item.name
            for item in fields(self)
            if self.slot(item.name) is not None
            and item.name not in {"vehicles", "enabled_tasks"}
        )
    @classmethod
    def from_slots(
        cls,
        slots: tuple[str, ...],
        *,
        vehicles: tuple[str, ...] = ("drone",),
        view_modes: tuple[str, ...] = (),
        weather_fields: dict[str, str] | None = None,
        weather_presets: tuple[str, ...] = (),
        enabled_tasks: tuple[str, ...] = (),
    ) -> SimulatorPorts:
        """A stand-in bundle built from the slot names a simulator declares.

        The console needs feature flags before any simulator is running, so the
        port list is reported without constructing clients. ``_Slots`` answers
        only what the derivation reads, and the test suite checks that the
        declared list matches what ``build_ports`` really returns.
        """
        present = set(slots)
        weather = _Weather(weather_fields or {}, weather_presets) if "weather" in present else None
        views = _Views(view_modes) if "views" in present else None
        return cls(
            vehicle=_Slot("vehicle") if "vehicle" in present else None,
            observation=_Slot("observation") if "observation" in present else None,
            streaming=_Slot("streaming") if "streaming" in present else None,
            controlled_capture=_Slot("controlled_capture") if "controlled_capture" in present else None,
            moving_capture=_Slot("moving_capture") if "moving_capture" in present else None,
            weather=weather,
            views=views,
            models=_Slot("models") if "models" in present else None,
            frames=_Slot("frames") if "frames" in present else None,
            home=_Slot("home") if "home" in present else None,
            vehicles=vehicles,
            enabled_tasks=enabled_tasks,
        )


class _Slot:
    """Placeholder for a declared-but-not-yet-connected port."""

    def __init__(self, name: str) -> None:
        self.name = name

    def view_modes(self) -> tuple[str, ...]:  # pragma: no cover - placeholder
        return ()

    def fields(self) -> dict[str, str]:  # pragma: no cover - placeholder
        return {}

    def presets(self) -> tuple[str, ...]:  # pragma: no cover - placeholder
        return ()


class _Views(_Slot):
    def __init__(self, modes: tuple[str, ...]) -> None:
        super().__init__("views")
        self._modes = modes

    def view_modes(self) -> tuple[str, ...]:
        return self._modes


class _Weather(_Slot):
    def __init__(self, fields: dict[str, str], presets: tuple[str, ...]) -> None:
        super().__init__("weather")
        self._fields = fields
        self._presets = presets

    def fields(self) -> dict[str, str]:
        return dict(self._fields)

    def presets(self) -> tuple[str, ...]:
        return self._presets
