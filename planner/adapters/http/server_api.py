"""Console and backend routes. The simulator and model calls stay off the HTTP handlers.

``POST /api/tasks`` accepts work only in ``idle``; ``POST /api/stop`` asks the
running task to abort between motion setpoints. ``POST /api/home`` returns the
selected vehicle to its spawn pose while idle, ``POST /api/vehicle`` switches
the active vehicle (park the other, home it, show its chase camera),
``POST /api/world`` restarts the simulator on another catalog map, and
``GET/POST /api/weather`` plus ``POST /api/view`` drive the simulator visual
state.
"""

from __future__ import annotations

from dataclasses import asdict

from fastapi import FastAPI, Query
from fastapi.responses import JSONResponse, StreamingResponse

from planner.assembly import ServerState
from planner.domain import default_view_for
from planner.runtime import TrajectorySnapshot
from .schemas import (
    HomeRequest,
    ModelsRequest,
    ProfileRequest,
    TaskRequest,
    VehicleRequest,
    ViewRequest,
    WorldRequest,
)
from .video import BOUNDARY, mjpeg_chunks
from .wiring import (
    build_app,
    capabilities,
    capability_gate,
    invoke,
    missing_catalog,
    mount_console,
)


def create_api(controller, settings, frames=None, trajectory_hub=None) -> FastAPI:
    """Build the console API.

    Every optional control is gated on the active map's ``capabilities``: an
    action the map's simulator does not offer returns 501 with
    ``detail.unsupported``. The flags come from the controller, which derives
    them from the simulator's port bundle, so a simulator cannot advertise a
    port it lacks and this layer names no simulator of its own.
    """

    app = build_app(controller, settings, frames)
    require = capability_gate(controller)

    @app.post("/api/tasks", status_code=202)
    def submit(request: TaskRequest):
        task = invoke(controller.submit, request.instruction, request.vehicle)
        return {
            "task_id": task.task_id,
            "state": ServerState.RUNNING.value,
            "vehicle": task.vehicle,
        }

    @app.get("/api/status")
    def status():
        payload = asdict(controller.status)
        caps, simulator = capabilities(controller)
        if caps is not None:
            payload["simulator"] = simulator
            payload["capabilities"] = caps.as_dict()
        return payload

    @app.get("/api/trajectory")
    def trajectory():
        # No capability gate: the remaining path is produced by the shared
        # workflows into this service's hub, for every simulator family.
        snapshot = (
            TrajectorySnapshot()
            if trajectory_hub is None
            else trajectory_hub.read()
        )
        return asdict(snapshot)

    @app.get("/api/logs")
    def logs(cursor: int = Query(default=0, ge=0)):
        return asdict(controller.logs.read(cursor))

    @app.post("/api/stop")
    def stop():
        state = controller.stop()
        code = 202 if state is ServerState.STOPPING else 200
        return JSONResponse({"state": state.value}, status_code=code)

    @app.post("/api/home", status_code=202)
    def home(request: HomeRequest | None = None):
        require("home")
        vehicle = request.vehicle if request is not None else None
        chosen = invoke(controller.home, vehicle)
        return {"state": ServerState.IDLE.value, "vehicle": chosen}

    @app.post("/api/vehicle", status_code=202)
    def activate_vehicle(request: VehicleRequest):
        # No vehicle whitelist here: ServerController rejects ids outside its
        # own roster, and the console filters the buttons with ``capabilities``.
        chosen = invoke(controller.activate, request.vehicle)
        return {
            "state": ServerState.IDLE.value,
            "vehicle": chosen,
            "view_mode": default_view_for(chosen),
        }

    @app.get("/api/world/options")
    def world_options():
        maps = getattr(controller, "maps", None)
        if maps is None:
            raise missing_catalog("map")
        current = controller.status
        caps, simulator = capabilities(controller)
        payload = {
            "maps": list(maps.ids),
            "map_labels": maps.labels,
            "default_map": maps.default_map,
            "selection": {"map": current.map},
            "simulator_owned": bool(
                getattr(controller, "simulator_owned", False)
            ),
        }
        if caps is not None:
            # The console renders controls from these flags and never tests a
            # simulator name, so a new simulator needs no frontend change.
            payload["map_simulators"] = {
                entry.id: entry.simulator for entry in maps.maps
            }
            payload["simulator_labels"] = controller.simulator_labels
            payload["simulator"] = simulator
            payload["capabilities"] = caps.as_dict()
        return payload

    @app.post("/api/world")
    def switch_world(request: WorldRequest):
        # The router picks the driver from the map's family, so a UE4 map can no
        # longer reach the ProjectAirSim launch script.
        state = invoke(controller.request_world, request.map)
        code = 202 if state is ServerState.CONNECTING else 200
        return JSONResponse(
            {"state": state.value, "map": request.map}, status_code=code
        )

    @app.post("/api/world/stop", status_code=202)
    def stop_world():
        # Service-level action: no capability gate. A simulator family states what
        # it can do inside the scene, not whether this service may take the whole
        # scene down; the controller refuses when it does not own the simulator.
        state = invoke(controller.stop_world)
        return JSONResponse({"state": state.value}, status_code=202)

    @app.get("/api/models")
    def model_options():
        require("models")
        catalog = getattr(controller, "vehicle_models", None)
        if catalog is None:
            raise missing_catalog("vehicle model")
        current = controller.status
        return {
            "models": dict(current.models or {}),
            "options": {
                group.vehicle: [
                    {"id": option.id, "label": option.label}
                    for option in group.options
                ]
                for group in catalog.groups
            },
            "switching": bool(current.models_switching),
        }

    @app.post("/api/models")
    def switch_models(request: ModelsRequest):
        require("models")
        state = invoke(controller.request_models, request.models)
        code = 202 if state is ServerState.CONNECTING else 200
        return JSONResponse(
            {
                "state": state.value,
                "models": dict(controller.status.models or {}),
            },
            status_code=code,
        )

    @app.get("/api/profile")
    def profile_options():
        # Service-level, like stopping the world: a profile picks the planner and the
        # navigation decisions, not what a simulator can do inside the scene, so no
        # capability gate stands in front of it.
        catalog = getattr(controller, "profiles", ())
        return {
            "current": controller.profile,
            "profiles": [
                {
                    "id": info.id,
                    "label": info.label,
                    "note": info.note,
                    "model": info.model,
                }
                for info in catalog
            ],
            "switching": bool(controller.status.profile_switching),
        }

    @app.post("/api/profile")
    def switch_profile(request: ProfileRequest):
        state = invoke(controller.request_profile, request.profile)
        code = 202 if state is ServerState.CONNECTING else 200
        return JSONResponse(
            {"state": state.value, "profile": request.profile}, status_code=code
        )

    @app.get("/api/weather")
    def weather():
        require("weather")
        catalog = getattr(controller, "weather_catalog", None)
        if catalog is None:
            raise missing_catalog("weather")
        return {
            "fields": [
                {"id": field.id, "label": field.label}
                for field in catalog.fields
            ],
            "presets": [
                {
                    "id": preset.id,
                    "label": preset.label,
                    "values": dict(preset.values),
                }
                for preset in catalog.presets
            ],
        }

    @app.post("/api/weather", status_code=202)
    def set_weather(payload: dict[str, object]):
        require("weather")
        merged = invoke(controller.request_weather, payload)
        return {
            "state": ServerState.IDLE.value,
            "weather": merged,
        }

    @app.post("/api/view")
    def set_view(request: ViewRequest):
        mode, deferred = invoke(controller.request_view, request.mode, code=422)
        return {"view_mode": mode, "deferred": deferred}


    if frames is not None:

        @app.get("/api/video")
        def video():
            # The frame hub only ever fills while the active family publishes.
            require("video")
            return StreamingResponse(
                mjpeg_chunks(frames),
                media_type=(
                    "multipart/x-mixed-replace; boundary="
                    + BOUNDARY.decode("ascii")
                ),
            )

    mount_console(app)

    return app