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

import json
import logging
import os
import re
import socket
import subprocess
import time
from contextlib import asynccontextmanager
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Literal

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, field_validator
from starlette.concurrency import run_in_threadpool

from eval.evalcore.config import REPO_ROOT, load_config as load_eval_config

from ...domain.server import BackendUnavailable, ServerState
from ...domain.trajectory_telemetry import TrajectorySnapshot
from ...domain.view import default_view_for
from ...simulator import registry
from .video import BOUNDARY, mjpeg_chunks

CONSOLE_DIR = Path(__file__).resolve().parents[3] / "console"

# 控制台点「评测」时，在 UE4 的 Python 环境中启动仪表盘。
_EVAL_DIR = REPO_ROOT / Path(os.environ.get("AVLN_EVAL_DIR") or "eval").expanduser()
_EVAL_LOCK = Lock()
_EVAL_PROC: tuple[subprocess.Popen, str, int] | None = None


def _slug(text: str) -> str:
    """把任务名变成安全的目录名（保留中文，其余转下划线）。"""
    cleaned = re.sub(r"[^\w\u4e00-\u9fff]+", "_", text or "").strip("_")
    return (cleaned or "task")[:40]


def _port_open(port: int, host: str = "127.0.0.1", timeout: float = 0.3) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        try:
            s.connect((host, port))
            return True
        except OSError:
            return False


def _eval_config():
    try:
        return load_eval_config()
    except (OSError, TypeError, ValueError) as exc:
        raise HTTPException(503, detail={"message": f"无法加载评测配置: {exc}"}) from exc


def _start_eval_service(planner_api_base: str, runtime_python: str) -> dict:
    """按需启动评测仪表盘，复用本进程已启动的子进程。"""
    global _EVAL_PROC
    log_path = _EVAL_DIR / "eval_serve.log"
    with _EVAL_LOCK:
        started = _EVAL_PROC is None or _EVAL_PROC[0].poll() is not None
        if started:
            _EVAL_PROC = None
            cfg = _eval_config()
            port = cfg.dashboard_port
            host = cfg.dashboard_host or "127.0.0.1"
            if host == "0.0.0.0":
                host = "127.0.0.1"
            if _port_open(port, host):
                raise HTTPException(409, detail={"message": f"评测端口 {port} 已被占用"})
            if not _EVAL_DIR.is_dir():
                raise HTTPException(503, detail={"message": f"评测目录不存在: {_EVAL_DIR}"})
            interpreter = os.environ.get("AVLN_EVAL_PYTHON") or runtime_python
            if not interpreter:
                raise HTTPException(503, detail={"message": "请配置 simulators.yaml 的 airsim.runtime_python"})
            python = REPO_ROOT / Path(interpreter).expanduser()
            if not python.is_file():
                raise HTTPException(503, detail={"message": f"评测解释器不存在: {python}"})
            with log_path.open("ab") as log:
                proc = subprocess.Popen(
                    [str(python), "-u", "run.py", "serve", "--config", str(cfg.config_path),
                     "--dashboard-port", str(port)],
                    cwd=_EVAL_DIR,
                    stdin=subprocess.PIPE,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env={**os.environ, "AVLN_PLANNER_API": planner_api_base,
                         "AVLN_EVAL_PARENT_PIPE": "1"},
                    start_new_session=True,
                )
            _EVAL_PROC = (proc, host, port)

        proc, host, port = _EVAL_PROC
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise HTTPException(
                    500,
                    detail={"message": f"评测服务已退出，见 {log_path}"},
                )
            if _port_open(port, host) and proc.poll() is None:
                return {"started": started, "port": port}
            time.sleep(0.3)
        raise HTTPException(
            504, detail={"message": f"评测服务未在 30s 内打开端口 {port}"}
        )


def _close_eval_service() -> None:
    global _EVAL_PROC
    with _EVAL_LOCK:
        if _EVAL_PROC is None:
            return
        proc, _, _ = _EVAL_PROC
        _EVAL_PROC = None
        proc.stdin.close()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            logging.getLogger(__name__).warning("评测仪表盘未在 5 秒内退出")


#: One process driver for every family: planner/simulator/xpra_start.py renders the
#: argv the family declares in config/simulators.yaml. ``SimulatorFamily`` rejects
#: any other driver when the config is parsed, so the HTTP layer does not guess
#: again and nothing here branches on a simulator name.

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


class ProfileRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    profile: str

    @field_validator("profile")
    @classmethod
    def nonempty(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("profile must be non-empty text")
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


def create_api(controller, settings, frames=None, trajectory_hub=None, *, eval_runtime_python="") -> FastAPI:
    """Build the console API.

    Every optional control is gated on the active map's ``capabilities``: an
    action the map's simulator does not offer returns 501 with
    ``detail.unsupported``. Those flags come from the simulator registry, not
    from a config file, so a simulator cannot advertise a port it lacks.
    """

    def capabilities():
        """Flags derived from the ports of the simulator driving the active map."""
        catalog = getattr(controller, "maps", None)
        entry = catalog.entry(controller.status.map) if catalog is not None else None
        if entry is None:
            return None, ""
        return registry.capabilities_for(entry.simulator), entry.simulator

    def require(action):
        caps, simulator = capabilities()
        if caps is not None and not caps.supports(action):
            raise HTTPException(
                501,
                detail={
                    "unsupported": action,
                    "simulator": simulator,
                    "message": (
                        f"simulator '{simulator}' does not implement the port "
                        f"behind {action}"
                    ),
                },
            )

    @asynccontextmanager
    async def lifespan(app):
        controller.start()
        try:
            yield
        finally:
            try:
                await run_in_threadpool(_close_eval_service)
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
            # Fresh flight history for the new task; the console's save button
            # reads it to snapshot each goal-bearing subtask after the run.
            if (
                trajectory_hub is not None
                and controller.status.state is ServerState.IDLE
            ):
                trajectory_hub.reset_history()
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
        payload = asdict(controller.status)
        caps, simulator = capabilities()
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
        # No vehicle whitelist here: ServerController rejects ids outside its
        # own roster, and the console filters the buttons with ``capabilities``.
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
        caps, simulator = capabilities()
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
            payload["simulator_labels"] = registry.labels()
            payload["simulator"] = simulator
            payload["capabilities"] = caps.as_dict()
        return payload

    @app.post("/api/world")
    def switch_world(request: WorldRequest):
        # The router picks the driver from the map's family, so a UE4 map can no
        # longer reach the ProjectAirSim launch script.
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

    @app.post("/api/world/stop", status_code=202)
    def stop_world():
        # Service-level action: no capability gate. A simulator family states what
        # it can do inside the scene, not whether this service may take the whole
        # scene down; the controller refuses when it does not own the simulator.
        try:
            state = controller.stop_world()
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        return JSONResponse({"state": state.value}, status_code=202)

    @app.get("/api/models")
    def model_options():
        require("models")
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
        require("models")
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
        try:
            state = controller.request_profile(request.profile)
        except BackendUnavailable as exc:
            raise _unavailable(exc) from exc
        except ValueError as exc:
            raise HTTPException(
                400, detail={"message": str(exc)}
            ) from exc
        code = 202 if state is ServerState.CONNECTING else 200
        return JSONResponse(
            {"state": state.value, "profile": request.profile}, status_code=code
        )

    @app.get("/api/weather")
    def weather():
        require("weather")
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
        require("weather")
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

    @app.get("/api/eval/pending")
    def eval_pending():
        # 控制台「保存执行记录」按钮据此启用：最近一次任务已结束、且含带目标子任务。
        summary = controller.last_task_summary
        stages = (summary or {}).get("stages") or []
        goal = [s for s in stages if s.get("goal")]
        state = controller.status.state.value
        return {
            "state": state,
            "has_task": summary is not None,
            "savable": summary is not None and bool(goal) and state == "idle",
            "goal_stages": [
                {
                    "stage_id": s.get("stage_id"),
                    "name": s.get("instruction") or s.get("target_description", ""),
                    "success": bool(s.get("success")),
                }
                for s in goal
            ],
        }

    @app.post("/api/eval/save", status_code=200)
    def save_eval():
        # 把最近一次任务里"带目标的子任务"逐条存成评测记录（轨迹+指标，按名字）。
        summary = controller.last_task_summary
        if not summary:
            raise HTTPException(409, detail={"message": "没有可保存的已完成任务"})
        stages = summary.get("stages") or []
        goal = [s for s in stages if s.get("goal")]
        if not goal:
            raise HTTPException(409, detail={"message": "该任务没有带目标的子任务"})
        history = [] if trajectory_hub is None else trajectory_hub.history()
        by_stage: dict[object, list] = {}
        for point in history:
            by_stage.setdefault(point.get("stage_id"), []).append(
                [point["x"], point["y"], point["z"]]
            )
        vehicle = controller.last_task_vehicle
        map_id = controller.status.map
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        saved_dir = Path(_eval_config().saved_dir)
        saved_dir.mkdir(parents=True, exist_ok=True)
        saved = []
        for stage in goal:
            stage_id = stage.get("stage_id")
            positions = by_stage.get(stage_id, [])
            name = (
                stage.get("instruction")
                or stage.get("target_description")
                or stage_id
                or "task"
            ).strip()
            success = bool(stage.get("success"))
            record = {
                "name": name,
                "instruction": stage.get("instruction", ""),
                "target_description": stage.get("target_description", ""),
                "kind": stage.get("kind", ""),
                "stage_id": stage_id,
                "sr": 1.0 if success else 0.0,
                "success": success,
                "final_distance_m": stage.get("final_distance_m"),
                "elapsed_s": stage.get("elapsed_s"),
                "path_length_m": stage.get("path_length_m"),
                "positions": positions,
                "start": positions[0] if positions else None,
                "vehicle": vehicle,
                "map": map_id,
                # 保存时用的正是主界面当前 profile（世界模型/普通 VLM），记进日志便于区分。
                "planner_profile": controller.profile,
                "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
            slug = _slug(name)
            rec_dir = saved_dir / f"{stamp}_{slug}"
            suffix = 0
            while rec_dir.exists():
                suffix += 1
                rec_dir = saved_dir / f"{stamp}_{slug}_{suffix}"
            rec_dir.mkdir(parents=True)
            (rec_dir / "record.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            saved.append({"name": name, "dir": rec_dir.name, "sr": record["sr"]})
        return {"saved": saved, "count": len(saved)}

    @app.post("/api/eval/start", status_code=200)
    def start_eval():
        # 懒启动评测仪表盘：控制台点「评测」先调这里，再打开返回的端口。
        # settings 是 ServerSettings（create_api 传的是 config.server）。
        host = settings.host
        if host in ("", "0.0.0.0"):
            host = "127.0.0.1"
        return _start_eval_service(f"http://{host}:{settings.port}", eval_runtime_python)


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