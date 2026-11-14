"""The child side of a mission session: build it, run commands, report events.

Started by :class:`planner.simulator.subprocess_worker.SubprocessMissionWorker`, never
by hand: it writes its protocol on the file descriptor the parent names in
:data:`~planner.simulator.subprocess_worker.EVENT_FD_ENV` and exits with a message when
that variable is missing, because a manually started child would otherwise sit reading
stdin while its parent waits for a ``READY`` that never comes.

Why a process at all is a dependency fact, recorded in ``docs/ENVIRONMENT.md`` section 1:
the AirSim client's ``msgpack-python`` and ProjectAirSim's ``msgpack`` own the same module
name, so the UE4 session cannot live in the interpreter that serves HTTP.

Everything below mirrors :class:`planner.simulator.worker.ThreadedMissionWorker` -- one
command queue, tasks on the owner thread, ``idle_tick`` between commands, the same event
kinds. The two must behave the same, because ``ServerController`` cannot tell them apart.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import dataclasses
import json
import logging
import os
import queue
import sys
import threading
from typing import Callable

from ..application.factories import (
    build_detector,
    build_geometry,
    build_planner,
)
from ..application.mission_assembly import MissionResources
from ..application.reporting import task_finish_events
from ..config import load_config
from ..domain.server import TaskSubmission, WorkerEvent, WorkerEventKind
from . import registry
from .subprocess_worker import EVENT_FD_ENV


def _protocol_fd() -> int:
    """The descriptor the parent told us to write on."""
    return int(os.environ.get(EVENT_FD_ENV, "0"))


def _jsonable(value):
    """Turn a DTO into something ``json.dumps`` accepts.

    The threaded worker hands ``ServerController`` live dataclasses (the weather
    catalog is one) and FastAPI encodes them at the edge. Crossing a pipe means the
    encoding happens here instead, and the console must not see a different shape
    because the session moved to a child process.
    """
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return dataclasses.asdict(value)
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    return value


class Emitter:
    """One JSON line per event on the descriptor the parent handed us."""

    def __init__(self, fd: int | None = None):
        number = _protocol_fd() if fd is None else fd
        if number <= 0:
            raise SystemExit(
                f"this process is not a mission child: ${EVENT_FD_ENV} is unset. "
                "Start it through SubprocessMissionWorker, not by hand -- a manually "
                "started child would sit reading stdin while its parent waits for a "
                "READY that can never arrive."
            )
        try:
            self._stream = os.fdopen(number, "w", encoding="utf-8", buffering=1)
        except OSError as error:
            raise SystemExit(
                f"mission child cannot open protocol descriptor {number}: {error}"
            ) from error
        self._lock = threading.Lock()

    def event(self, kind: WorkerEventKind, **fields) -> None:
        payload = {"kind": kind.value}
        payload.update({key: _jsonable(value) for key, value in fields.items()
                        if value is not None})
        line = json.dumps(payload, ensure_ascii=False)
        with self._lock:
            self._stream.write(line + "\n")
            self._stream.flush()

    def log(self, message: str, level: str = "INFO") -> None:
        self.event(WorkerEventKind.LOG, message=str(message)[:2000], level=level)

    def close(self) -> None:
        try:
            self._stream.close()
        except OSError:
            pass


class _LogBridge(logging.Handler):
    """Forward library logging into the protocol.

    ``airsim`` and ``msgpackrpc`` report failures through the root logger, and a
    mission child has no terminal attached: without this, the only trace of a
    simulator-side problem is an exit code.
    """

    def __init__(self, emit: Callable[[str, str], None]):
        super().__init__(level=logging.INFO)
        self._emit = emit
        self.setFormatter(logging.Formatter("%(name)s: %(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._emit(self.format(record), record.levelname)
        except Exception:
            pass


def _where(error: BaseException) -> str:
    """The deepest frame of a traceback, as ``file.py:line``.

    The child has no terminal and no log file: its stderr is forwarded as text, but an
    exception that is caught and reported as a one-line message leaves the operator
    with a type name and nothing to open. Measured here, that turned
    "'NoneType' object has no attribute 'run'" into a hunt through the whole session.
    """
    frame = None
    tb = error.__traceback__
    while tb is not None:
        frame = tb
        tb = tb.tb_next
    if frame is None:
        return ""
    return f" [at {Path(frame.tb_frame.f_code.co_filename).name}:{frame.tb_lineno}]"


class MissionChild:
    """Reads commands, owns the session, writes events."""

    def __init__(self, config_path: str, map_id: str):
        self._config = load_config(config_path)
        self._map_id = map_id
        self.emitter = Emitter()
        self._commands: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._stop_requested = False
        self._lock = threading.Lock()
        self._task_running: str | None = None
        self._task_pending: str | None = None
        self._session = None
        self._closing = False

    def _clear_running(self, task_id: str) -> None:
        """A finished task owns no stop flag and no running id.

        The threaded worker clears the flag after every task for the same reason: the
        stop that cancelled this task refers to this task. Left set, the next
        instruction started already cancelled -- measured on Blocks, where the flight
        after an abort was refused the moment it began.
        """
        with self._lock:
            if self._task_running == task_id:
                self._task_running = None
            self._task_pending = None
            self._stop_requested = False

    # -- plumbing ----------------------------------------------------------
    def log(self, message: str, level: str = "INFO") -> None:
        self.emitter.log(message, level)

    def _reader(self) -> None:
        for line in iter(sys.stdin.readline, ""):
            line = line.strip()
            if not line:
                continue
            try:
                command = json.loads(line)
            except ValueError as error:
                self.log(f"ignoring an unparseable command: {error}", "ERROR")
                continue
            if command.get("cmd") == "stop":
                # Out of band, on this thread. Tasks run on the command loop, so a
                # queued stop would only be reached after the flight it was meant to
                # cancel -- measured on Blocks, where /api/stop six seconds into an
                # eight-metre leg reported the stage completed. The threaded worker
                # sets its event directly, which is the behaviour the console is
                # written against.
                #
                # A stop names the work in front of the vehicle, never the next
                # instruction: applied to a task that already ended, or deferred to a
                # task not yet submitted, it refuses flights nobody asked to cancel.
                with self._lock:
                    if self._task_running is not None:
                        self._stop_event.set()
                        target = self._task_running
                    elif self._task_pending is not None:
                        self._stop_requested = True
                        target = f"{self._task_pending} (queued)"
                    else:
                        target = None
                self.log(
                    f"stop requested for {target}" if target
                    else "stop requested with nothing flying; ignored",
                    "WARNING",
                )
                continue
            if command.get("cmd") == "task":
                # Claim it before it reaches the queue: a stop can arrive from the pipe
                # at any moment, and "queued" is already a task the operator can cancel.
                with self._lock:
                    self._task_pending = str(command.get("task_id", ""))
            self._commands.put(command)
        self._commands.put({"cmd": "terminate"})   # parent closed the pipe

    def _build_session(self):
        config = self._config
        entry = config.maps.entry(self._map_id)
        if entry is None:
            raise ValueError(
                f"map {self._map_id!r} is not in the catalog; known maps are "
                f"{list(config.maps.ids)}"
            )
        family = config.families.require(entry.simulator)
        resources = MissionResources(
            detector=build_detector(
                config,
                config.server.detector,
                bbox=config.server.manual_bbox,
                surface_points=config.server.manual_surface_points,
            ),
            planner=build_planner(config, config.server.planner),
            geometry=build_geometry(config),
        )
        self.log(f"Connecting to {family.label} for the mission child.")
        return registry.build_session(
            entry.simulator,
            config=config,
            map_entry=entry,
            family=family,
            resources=resources,
            log=self.log,
        )

    # -- command handling --------------------------------------------------
    def _run_task(self, command: dict) -> None:
        task = TaskSubmission(
            task_id=str(command.get("task_id", "")),
            instruction=str(command.get("instruction", "")),
            vehicle=str(command.get("vehicle", "") or ""),
        )
        with self._lock:
            self._stop_event = threading.Event()
            if self._stop_requested:
                self._stop_event.set()      # a stop that arrived while the task queued
            self._stop_requested = False
            self._task_pending = None
            stop_event = self._stop_event
            self._task_running = task.task_id
        self.log(f"Task {task.task_id} started: {task.instruction}")
        try:
            result = self._session.run(task, stop_event=stop_event, log=self.log)
        except Exception as error:            # keep the child alive for the next task
            message = f"Task rejected: {type(error).__name__}: {error}{_where(error)}"
            self.log(message, "ERROR")
            self.emitter.event(
                WorkerEventKind.FINISHED, task_id=task.task_id, message=message, success=False
            )
            self._clear_running(task.task_id)
            return
        lines, event = task_finish_events(result, task.task_id)
        for line in lines:
            self.log(line)
        self.emitter.event(event.kind, **{
            key: value for key, value in dataclasses.asdict(event).items() if key != "kind"
        })
        self._clear_running(task.task_id)

    def _handle(self, command: dict) -> bool:
        """Apply one command; return False when the child should exit."""
        cmd = command.get("cmd")
        if cmd == "task":
            self._run_task(command)
        elif cmd == "stop":
            with self._lock:
                self._stop_event.set()
        elif cmd == "weather":
            try:
                self._session.set_weather(command.get("fields") or {})
                # The session already logs what it applied; a second copy here made
                # every weather change read as two identical lines.
            except Exception as error:
                self.log(f"weather failed: {type(error).__name__}: {error}", "ERROR")
        elif cmd in ("shutdown", "terminate"):
            skip = bool(command.get("skip_landing")) or cmd == "terminate"
            try:
                self._session.close(skip_landing=skip)
            except Exception as error:
                self.log(f"session close failed: {error}", "WARNING")
            return False
        elif cmd in ("home", "activate", "view"):
            # The console may only send these when the capability report offers them,
            # and a family that answers them needs no code in this loop.
            self.log(
                f"the mission child has no {cmd!r} route for this simulator",
                "ERROR",
            )
        else:
            self.log(f"unknown command {cmd!r} ignored", "ERROR")
        return True

    # -- main --------------------------------------------------------------
    def run(self) -> int:
        bridge = _LogBridge(self.log)
        logging.getLogger().addHandler(bridge)
        logging.getLogger().setLevel(logging.INFO)
        threading.Thread(target=self._reader, name="mission-child-stdin", daemon=True).start()
        try:
            self._session = self._build_session()
        except Exception as error:
            self.emitter.event(
                WorkerEventKind.ERROR,
                message=f"mission session setup failed: {type(error).__name__}: {error}",
            )
            self.emitter.event(WorkerEventKind.CLOSED, message="mission child closed")
            return 1
        try:
            self.emitter.event(WorkerEventKind.READY)
            catalog = getattr(self._session, "weather_catalog", None)
            if catalog is not None:
                try:
                    self.emitter.event(
                        # The wire form is the domain type's own, so the parent can hand
                        # back a WeatherCatalog instead of a dict of one.
                        WorkerEventKind.WEATHER, payload=catalog().to_payload()
                    )
                except Exception as error:
                    self.log(f"weather catalog unavailable: {error}", "WARNING")
            idle_tick = getattr(self._session, "idle_tick", None)
            while True:
                try:
                    command = self._commands.get(timeout=0.05)
                except queue.Empty:
                    if idle_tick is not None:
                        idle_tick()
                    continue
                if not self._handle(command):
                    break
        finally:
            logging.getLogger().removeHandler(bridge)
            if self._session is not None:
                try:
                    self._session.close(skip_landing=True)
                except Exception as error:
                    self.log(f"session close on exit failed: {error}", "WARNING")
            self.emitter.event(WorkerEventKind.CLOSED, message="mission child closed")
            self.emitter.close()
        return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="path to config/base.yaml")
    parser.add_argument("--map", required=True, help="map id from config/maps.yaml")
    args = parser.parse_args(argv)
    return MissionChild(args.config, args.map).run()


if __name__ == "__main__":
    sys.exit(main())
