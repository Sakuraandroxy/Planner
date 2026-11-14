"""The bundle a simulator hands the shared layer, and what it implies.

``SimulatorPorts`` is the only seam between the task logic and a simulator. The
shared layer asks for a port and either gets an object or gets ``None``; it
never asks which simulator is running.

``capabilities()`` derives the console's feature flags from those slots. That is
deliberate: a config file that lists capabilities can drift from what the code
implements, and the drift only shows up as a broken button on a machine with a
simulator attached.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

from ..domain.runtime import Capabilities
from ..ports.capabilities import (
    AppearanceModelControl,
    CameraView,
    FramePublisher,
    ReturnHome,
    StreamingViews,
    TrajectoryTelemetry,
    WeatherControl,
)
from ..ports.observation_source import (
    ControlledObservationSource,
    MovingObservationSource,
    ObservationSource,
)
from ..ports.streaming_vehicle import StreamingVehicle
from ..ports.vehicle import Vehicle

#: Task kind -> port slots it cannot run without. Keys are ``SimulatorPorts``
#: field names, so adding a task means adding one row, not one branch.
TASK_REQUIREMENTS: dict[str, tuple[str, ...]] = {
    "basic_motion": ("vehicle",),
    "visual_goal_navigation": ("vehicle", "observation"),
    "free_navigation": ("vehicle", "observation", "streaming"),
    "visual_goal_above": ("vehicle", "observation", "camera_view"),
}

#: Port slot -> the ``actions`` flag it unlocks.
ACTION_PORTS: dict[str, str] = {
    "home": "home",
    "models": "models",
    "video": "frames",
    "trajectory": "telemetry",
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
    camera_view: CameraView | None = None
    weather: WeatherControl | None = None
    views: StreamingViews | None = None
    models: AppearanceModelControl | None = None
    frames: FramePublisher | None = None
    telemetry: TrajectoryTelemetry | None = None
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
