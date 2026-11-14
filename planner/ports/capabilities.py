"""Optional capability ports: one interface per simulator-specific behaviour.

A simulator either implements one of these or leaves it out of its
``SimulatorPorts``. Absence is a normal state, not an error: it is what makes
"UE4 can look down, UE5 can swap appearance models" a data-free statement about
which interfaces each simulator actually wrote.

Do not add methods to ``planner.ports.vehicle`` or ``backend_worker`` to express
a new capability. Add a port here, and have the simulators that can honour it
implement it. Four capabilities (home, view switching, appearance models,
weather) used to be crammed into ``BackendWorker`` as ten flat methods, so an
implementer could only answer them by raising or returning nothing.
"""

from __future__ import annotations

from typing import Protocol


class CameraView(Protocol):
    """Repoint a body-mounted camera. AirSim exposes this, ProjectAirSim does not.

    The six methods are the surface ``adapters/airsim/above_camera.py`` already
    implements: save the factory mount, aim, restore.
    """

    def read_mount(self) -> object: ...

    def point(self, pitch_deg: float, yaw_deg: float = 0.0) -> None: ...

    def point_at_world(self, point: tuple[float, float, float]) -> None: ...

    def point_direction_world(
        self, direction: tuple[float, float, float]
    ) -> None: ...

    def point_down(self) -> None: ...

    def restore(self, mount: object) -> None: ...


class ReturnHome(Protocol):
    """Drive the selected vehicle back to its spawn pose while idle."""

    def return_home(self, vehicle: str) -> None: ...


class WeatherControl(Protocol):
    """Apply and describe the simulator's weather parameters.

    ``fields`` maps the planner's canonical names to the spelling this
    simulator uses, so the ``road_wetness`` against ``roadwetness`` difference
    lives in the adapter that owns it instead of in a config alias table the
    simulator could lie about.
    """

    def fields(self) -> dict[str, str]: ...

    def presets(self) -> tuple[str, ...]: ...

    def apply(self, values: dict[str, float]) -> dict[str, float]: ...


class StreamingViews(Protocol):
    """Cycle the simulator's rendered camera. Absent means no view buttons."""

    def view_modes(self) -> tuple[str, ...]: ...

    def set_view_mode(self, mode: str) -> bool:
        """Return True when applied now, False when deferred to task end."""


class AppearanceModelControl(Protocol):
    """Swap the packaged mesh a vehicle is rendered with.

    ``options`` is reported by the simulator so the console cannot offer a model
    the running scene does not contain.
    """

    def options(self) -> dict[str, tuple[tuple[str, str], ...]]: ...

    def current(self) -> dict[str, str]: ...

    def request(self, models: dict[str, str]) -> bool:
        """Return True when a scene reload is needed to apply the change."""


class FramePublisher(Protocol):
    """Stream captured frames to the browser. ``/api/video`` is gated on this."""

    def publish(self, name: str, jpeg: bytes, timestamp_ns: int) -> None: ...


class TrajectoryTelemetry(Protocol):
    """Report the remaining world waypoints the control loop is following."""

    def snapshot(self) -> object: ...


class VehicleRoster(Protocol):
    """Which vehicles this simulator actually spawns and can control."""

    @property
    def vehicles(self) -> tuple[str, ...]: ...

    @property
    def default_vehicle(self) -> str: ...

