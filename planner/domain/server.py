"""HTTP-backend lifecycle types; no framework or process objects."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ServerState(str, Enum):
    # No scene is loaded and no simulator connection is attempted until the
    # console asks for a map (the planner's startup state).
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    RUNNING = "running"
    STOPPING = "stopping"


class WorkerEventKind(str, Enum):
    LOG = "log"
    READY = "ready"
    FINISHED = "finished"
    ERROR = "error"
    CLOSED = "closed"
    WEATHER = "weather"
    ACTIVE = "active"
    VIEW = "view"


@dataclass(frozen=True)
class MapSelection:
    """Scene identity handed to the worker factory for a map."""

    map_id: str
    scene_file: str
    sim_config_path: str


@dataclass(frozen=True)
class TaskSubmission:
    task_id: str
    instruction: str
    # The session routes the task to this vehicle id ("rover" or "drone").
    vehicle: str = ""


@dataclass(frozen=True)
class ServerStatus:
    state: ServerState
    instruction: str = ""
    last_result: str | None = None
    vehicle: str = ""
    vehicles: tuple[str, ...] = ()
    # Selected/active map id and every id the console may switch to.
    map: str = ""
    maps: tuple[str, ...] = ()
    # Active streaming view id ("" until the session reports one).
    view_mode: str = ""
    # True while the simulator is stepping through streaming views to apply a
    # requested camera; clients show a blank screen instead of the switching.
    view_pending: bool = False
    # Complete eight-field weather state, or None before the session is ready.
    weather: dict[str, float] | None = None
    # True while the simulator stack is being stopped/relaunched for a map.
    map_switching: bool = False
    # Selected appearance model per vehicle, or None without a model catalog.
    models: dict[str, str] | None = None
    # True while the worker is restarting to load a model's scene variant.
    models_switching: bool = False
    # Last simulator restart failure, surfaced by /api/status and /api/logs.
    sim_error: str | None = None



@dataclass(frozen=True)
class WorkerEvent:
    kind: WorkerEventKind
    task_id: str | None = None
    message: str = ""
    level: str = "INFO"
    success: bool | None = None
    payload: object | None = None
    # Set by ACTIVE and VIEW events: the vehicle/view the worker applied.
    vehicle: str | None = None


@dataclass(frozen=True)
class ServerLog:
    id: int
    time: str
    level: str
    message: str


@dataclass(frozen=True)
class LogBatch:
    items: tuple[ServerLog, ...]
    next_cursor: int
    truncated: bool = False


class BackendUnavailable(RuntimeError):
    def __init__(self, state: ServerState):
        self.state = state
        super().__init__(f"planner is {state.value}")