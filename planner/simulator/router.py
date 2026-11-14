"""Route one controller to the driver that owns each map's simulator family.

``ServerController`` holds a single supervisor and calls it with no map in hand
(``running()``, ``preflight()``, ``stop()``). Two families now share the service,
and they are started and stopped differently: ProjectAirSim goes through the
launch script, AirSim through a rendered ``xpra start`` argv. This object picks
the driver by family and keeps the controller's five-method view intact.

Calls that carry a map route by ``entry.simulator``. Calls that do not route by
the map the marker file names, which is the same record the boot sequence reads,
so a service restart still finds the stack that is actually up.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from ..config import MapCatalog, MapEntry, SimulatorSettings
from .supervisor import read_map_marker
from .xpra_start import supervisor_for

LogFn = Callable[[str, str], None]


class FamilyRouter:
    """One supervisor facade over one driver per simulator family."""

    def __init__(
        self,
        families,
        settings: SimulatorSettings,
        project_dir: str | Path,
        maps: MapCatalog,
        log: LogFn | None = None,
        **kwargs,
    ):
        self._families = families
        self._settings = settings
        self._maps = maps
        self._project = Path(project_dir).resolve()
        self._log_fn = log
        self._kwargs = kwargs
        self._drivers: dict[str, object] = {}

    # -- driver selection --------------------------------------------------
    def _driver(self, family_id: str):
        driver = self._drivers.get(family_id)
        if driver is None:
            family = self._families.entry(family_id)
            if family is None:
                raise ValueError(
                    f"map family {family_id!r} is not configured in "
                    f"config/simulators.yaml; known families are {list(self._families.ids)}"
                )
            driver = supervisor_for(
                family, self._settings, self._project, self._maps, log=self._log_fn, **self._kwargs
            )
            self._drivers[family_id] = driver
        return driver

    def _active_family_id(self) -> str:
        """The family of the map the marker file says is running."""
        marker = Path(self._settings.map_marker_file)
        if not marker.is_absolute():
            marker = self._project / marker
        entry_id = read_map_marker(marker, self._maps)
        entry = self._maps.entry(entry_id) if entry_id else None
        return (entry.simulator if entry is not None else "") or self._families.default_family

    # -- controller-facing API --------------------------------------------
    def running(self) -> bool:
        return bool(self._driver(self._active_family_id()).running())

    def preflight(self) -> None:
        self._driver(self._active_family_id()).preflight()

    def stop(self) -> None:
        self._driver(self._active_family_id()).stop()

    def start(self, entry: MapEntry) -> None:
        self._driver(entry.simulator).start(entry)

    def restart(self, entry: MapEntry) -> None:
        self._driver(entry.simulator).restart(entry)

    def launch_command(self, entry: MapEntry) -> str:
        """The command an operator would run by hand for this map."""
        return self._driver(entry.simulator).launch_command(entry)
