"""Immutable HTTP-backend lifecycle messages; no process or framework objects."""
from dataclasses import dataclass
from enum import Enum


class ServerState(str, Enum):
    CONNECTING = "connecting"
    IDLE = "idle"
    RUNNING = "running"
    STOPPING = "stopping"


@dataclass(frozen=True)
class ServerStatus:
    state: ServerState
    instruction: str


class WorkerMode(str, Enum):
    PREPARE = "prepare"
    HOLD = "hold"
    CLEANUP = "cleanup"


class WorkerEventKind(str, Enum):
    LOG = "log"
    READY = "ready"
    FINISHED = "finished"
    DISCONNECTED = "disconnected"
    ERROR = "error"
    CLOSED = "closed"
    EXITED = "exited"


@dataclass(frozen=True)
class TaskSubmission:
    task_id: str
    instruction: str


@dataclass(frozen=True)
class WorkerEvent:
    kind: WorkerEventKind
    task_id: str | None = None
    message: str = ""
    level: str = "INFO"
    success: bool | None = None


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
        super().__init__(f"Planner is {state.value}.")
