"""后端 worker 的内部协议：装配层按这组方法驱动进程栈，不关心实现形态。

线程 worker 与子进程 worker 都实现它。这是进程栈的协议，不是载具能力，
所以不留在 ``planner/ports/``。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class WorkerEventKind(str, Enum):
    LOG = "log"
    READY = "ready"
    FINISHED = "finished"
    ERROR = "error"
    CLOSED = "closed"
    WEATHER = "weather"
    ACTIVE = "active"
    VIEW = "view"
    #: The task loop lives in the child for families whose simulator client cannot
    #: be imported here, so the remaining-trajectory snapshots it publishes have to
    #: ride this channel to reach the hub the HTTP thread reads. Not a simulator
    #: capability: the console's trajectory panel exists for every family.
    TRAJECTORY = "trajectory"


@dataclass(frozen=True)
class TaskSubmission:
    task_id: str
    instruction: str
    # The session routes the task to this vehicle id ("rover" or "drone").
    vehicle: str = ""


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


class BackendWorker(Protocol):
    def start(self) -> None: ...

    def submit(self, task: TaskSubmission) -> None: ...

    def stop(self) -> None: ...

    def home(self, vehicle: str | None = None) -> None: ...

    def activate(self, vehicle: str) -> None: ...

    def set_view_mode(self, mode: str) -> None: ...

    def set_weather(self, fields: dict[str, float]) -> None: ...

    def poll(self) -> WorkerEvent | None: ...

    def shutdown(self, *, skip_landing: bool = False) -> None: ...

    def terminate(self) -> None: ...
