"""Appearance models per vehicle: listing and switching.

Changing a model reloads the scene in the same simulator process; a map switch
keeps the current selection.
"""

from __future__ import annotations

from ...config import VehicleModelCatalog
from .state import BackendUnavailable, ServerState


class ModelSwitcher:
    def __init__(self, controller):
        self._c = controller

    @property
    def vehicle_models(self) -> VehicleModelCatalog | None:
        return self._c._model_catalog

    def request_models(self, updates: dict[str, str]) -> ServerState:
        """Select appearance models and reload the scene (idle only).

        The simulator process and the loaded cooked map stay untouched; only
        the worker is restarted so the new session loads the matching scene
        variant. Every vehicle resets to its spawn pose.
        """
        c = self._c
        catalog = c._model_catalog
        if catalog is None:
            raise ValueError("this deployment does not configure vehicle models")
        if not isinstance(updates, dict) or not updates:
            raise ValueError("models must name at least one vehicle")
        cleaned: dict[str, str] = {}
        for vehicle, model_id in updates.items():
            vehicle_id = str(vehicle).strip()
            group = catalog.group(vehicle_id)
            if group is None:
                raise ValueError(
                    f"unknown vehicle '{vehicle_id}'; expected one of "
                    f"{list(catalog.vehicles)}"
                )
            chosen = str(model_id).strip()
            if group.option(chosen) is None:
                raise ValueError(
                    f"unknown {vehicle_id} model '{chosen}'; expected one of "
                    f"{list(group.ids)}"
                )
            cleaned[vehicle_id] = chosen
        with c._lock:
            if (
                c._closing.is_set()
                or c._state is not ServerState.IDLE
                or c._worker is None
                or not c._worker_ready
            ):
                raise BackendUnavailable(c._state)
            merged = dict(c._models)
            merged.update(cleaned)
            if merged == c._models:
                return c._state
            c._models = merged
            c._models_switching = True
            c._sim_error = None
            c._active = None
            c._instruction = ""
            c._last_result = None
            c._pending_activation = None
            c._view_pending = False
            c._state = ServerState.CONNECTING
            c._commands.put(("models", c._generation, None))
        c.lifecycle.log(
            "Switching vehicle models to "
            + ", ".join(f"{vehicle}={model}" for vehicle, model in cleaned.items())
            + "; reloading the scene."
        )
        return ServerState.CONNECTING

    def switch_models(self) -> None:
        """Worker CLOSED -> reconnect with the variant scene, same simulator."""
        c = self._c
        if not c.commands.close_worker(skip_landing=True):
            c.commands.failed(
                "The worker did not confirm shutdown for the vehicle-model "
                "switch. Retrying the session."
            )
            return
        with c._lock:
            c._models_switching = False
            c._selection = c.world.selection_for(c._map_id)
        c.lifecycle.log(
            "Vehicle models switched: "
            + ", ".join(
                f"{vehicle}={model}" for vehicle, model in c._models.items()
            )
            + f"; loading {c._selection.scene_file}."
        )
        c.commands.connect()
