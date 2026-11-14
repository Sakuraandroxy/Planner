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

from contextlib import asynccontextmanager

from dataclasses import asdict
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, field_validator
from starlette.concurrency import run_in_threadpool

from ...domain.server import BackendUnavailable, ServerState
from ...domain.trajectory_telemetry import TrajectorySnapshot
from ...domain.view import default_view_for
from .video import BOUNDARY, mjpeg_chunks

CONSOLE_DIR = Path(__file__).resolve().parents[3] / "console"

ViewMode = Literal["rover_front", "rover_chase", "drone_front", "drone_chase"]


def _unavailable(exc: BackendUnavailable) -> HTTPException:
    code = (
        503
        if exc.state in (ServerState.CONNECTING, ServerState.DISCONNECTED)
        else 409
    )
    return HTTPException(
        code, detail={"state": exc.state.value, "message": str(exc)}
    )


class TaskRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    instruction: str
    # Route the task to one of the session's vehicles ("rover" or "drone").
    vehicle: str | None = None

    @field_validator("instruction")
    @classmethod
    def nonempty(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("instruction must be non-empty text")
        return value.strip()

    @field_validator("vehicle")
    @classmethod
    def optional_vehicle(cls, value: str | None) -> str | None:
        if value is None:
            return None
        text = value.strip()
        return text or None


class WorldRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    map: str

    @field_validator("map")
    @classmethod
    def nonempty(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("map must be non-empty text")
        return value.strip()


class ViewRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mode: ViewMode


class HomeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    vehicle: str | None = None

    @field_validator("vehicle")
    @classmethod
    def optional_vehicle(cls, value: str | None) -> str | None:
        if value is None:
            return None
        text = value.strip()
        return text or None


class VehicleRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    vehicle: str

    @field_validator("vehicle")
    @classmethod
    def nonempty(cls, value: str) -> str:
        text = value.strip()
        if not text:
            raise ValueError("vehicle must be non-empty text")
        return text


class ModelsRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    # Partial updates are merged over the current selection.
    models: dict[str, str]

    @field_validator("models")
    @classmethod
    def nonempty(cls, value: dict[str, str]) -> dict[str, str]:
        cleaned = {}
        for vehicle, model_id in value.items():
            key = str(vehicle).strip()
            chosen = str(model_id).strip()
            if not key or not chosen:
                raise ValueError("models must name non-empty vehicles and ids")
            cleaned[key] = chosen
        if not cleaned:
            raise ValueError("models must name at least one vehicle")
        return cleaned


def create_api(controller, settings, frames=None, trajectory_hub=None) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        controller.start()
        try:
            yield
        finally:
            await run_in_threadpool(controller.close)

    app = FastAPI(title="ProjectAirSim VLN Planner API", lifespan=lifespan)
    app.state.frames = frames
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(settings.allowed_origins),
        allow_methods=["GET", "POST"],
        allow_headers=["Content-Type"],
        allow_credentials=False,
    )

    @app.middleware("http")
    async def cache_headers(request, call_next):
        response = await call_next(request)
        path = request.url.path
        if path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-store"
        elif path == "/" or path.startswith("/console/"):
            # Revalidate so an updated console never runs from a stale cache.
            response.headers["Cache-Control"] = "no-cache"
        return response

    @app.post("/api/tasks", status_code=202)
    def submit(request: TaskRequest):
        try:
            task = controller.submit(request.instruction, request.vehicle)
        except BackendUnavailable as exc:
            code = (
                503
                if exc.state
                in (ServerState.CONNECTING, ServerState.DISCONNECTED)
                else 409
            )
            raise HTTPException(
                code,
                detail={"state": exc.state.value, "message": str(exc)},
            ) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        return {
            "task_id": task.task_id,
            "state": ServerState.RUNNING.value,
            "vehicle": task.vehicle,
        }

    @app.get("/api/status")
    def status():
        return asdict(controller.status)

    @app.get("/api/trajectory")
    def trajectory():
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
        vehicle = request.vehicle if request is not None else None
        try:
            chosen = controller.home(vehicle)
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        return {"state": ServerState.IDLE.value, "vehicle": chosen}

    @app.post("/api/vehicle", status_code=202)
    def activate_vehicle(request: VehicleRequest):
        try:
            chosen = controller.activate(request.vehicle)
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        return {
            "state": ServerState.IDLE.value,
            "vehicle": chosen,
            "view_mode": default_view_for(chosen),
        }

    @app.get("/api/world/options")
    def world_options():
        maps = getattr(controller, "maps", None)
        if maps is None:
            raise HTTPException(
                503, detail={"message": "map catalog is unavailable"}
            )
        current = controller.status
        return {
            "maps": list(maps.ids),
            "map_labels": maps.labels,
            "default_map": maps.default_map,
            "selection": {"map": current.map},
            "simulator_owned": bool(
                getattr(controller, "simulator_owned", False)
            ),
        }

    @app.post("/api/world")
    def switch_world(request: WorldRequest):
        try:
            state = controller.request_world(request.map)
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        code = 202 if state is ServerState.CONNECTING else 200
        return JSONResponse(
            {"state": state.value, "map": request.map}, status_code=code
        )

    @app.get("/api/models")
    def model_options():
        catalog = getattr(controller, "vehicle_models", None)
        if catalog is None:
            raise HTTPException(
                503, detail={"message": "vehicle model catalog is unavailable"}
            )
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
        try:
            state = controller.request_models(request.models)
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        code = 202 if state is ServerState.CONNECTING else 200
        return JSONResponse(
            {
                "state": state.value,
                "models": dict(controller.status.models or {}),
            },
            status_code=code,
        )

    @app.get("/api/weather")
    def weather():
        catalog = getattr(controller, "weather_catalog", None)
        if catalog is None:
            raise HTTPException(
                503, detail={"message": "weather catalog is unavailable"}
            )
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
        try:
            merged = controller.request_weather(payload)
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        return {
            "state": ServerState.IDLE.value,
            "weather": merged,
        }

    @app.post("/api/view")
    def set_view(request: ViewRequest):
        try:
            mode, deferred = controller.request_view(request.mode)
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                422, detail={"message": str(exc)}
            ) from exc
        return {"view_mode": mode, "deferred": deferred}


    if frames is not None:

        @app.get("/api/video")
        def video():
            return StreamingResponse(
                mjpeg_chunks(frames),
                media_type=(
                    "multipart/x-mixed-replace; boundary="
                    + BOUNDARY.decode("ascii")
                ),
            )

    if CONSOLE_DIR.is_dir():
        app.mount(
            "/console",
            StaticFiles(directory=str(CONSOLE_DIR)),
            name="console",
        )

        @app.get("/", include_in_schema=False)
        def index():
            return FileResponse(CONSOLE_DIR / "index.html")

    return app