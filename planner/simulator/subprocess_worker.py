"""Run a mission session in a child process with its own simulator interpreter.

Why this exists is a dependency fact, not a preference: the AirSim client pulls
``msgpack-python`` 0.5.6, which owns the module name ``msgpack`` that
``projectairsim`` needs at 1.2.4, and the HTTP service runs in the interpreter that has
``projectairsim`` installed (``docs/ENVIRONMENT.md`` section 1). UE4 therefore has to run
somewhere else, and the only "somewhere else" that shares no ``sys.modules`` is a process.
A family says so with ``runtime_python`` in ``config/simulators.yaml``; a family without
it keeps the in-process :class:`~planner.simulator.worker.ThreadedMissionWorker`.

The interface is deliberately identical to the threaded worker: ``ServerController`` calls
``start / submit / stop / poll / shutdown / terminate`` plus the four optional commands,
and it must not care which side of a pipe the session lives on.

Protocol: one JSON object per line on a dedicated file descriptor (``EVENT_FD``, 3)
inherited by the child. Not stdout: a dependency printing one banner line to stdout
would be parsed as an event and raise a protocol error over nothing being wrong, so
stdout and stderr are relayed as raw log lines instead.

Commands are one JSON object per line on the child's stdin. The interface is
deliberately identical to the threaded worker: ``ServerController`` calls
``start / submit / stop / poll / shutdown / terminate`` plus the four optional
commands, and it must not care which side of a pipe the session lives on.
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import threading
from typing import Callable

from ..domain.server import TaskSubmission, WorkerEvent, WorkerEventKind

#: How long the child may take to exit after it is asked nicely.
TERM_GRACE_S = 10.0

#: Environment variable naming the file descriptor the child writes its protocol on.
#: The number cannot be a constant: ``os.pipe`` returns whatever is free in the parent,
#: and on a process with only stdio open that is 3 for the *read* end. Telling the
#: child which number it got is what keeps the channel out of stdout, where a stray
#: print from any library would otherwise be parsed as an event.
EVENT_FD_ENV = "PLANNER_MISSION_EVENT_FD"


class ChildProcessError(RuntimeError):
    """The child died or answered in something other than the protocol."""


def encode_command(cmd: str, **fields) -> str:
    """One protocol line for the child.

    ``TaskSubmission`` is a dataclass and the payload has to survive a JSON round trip,
    so task fields are flattened here instead of serialising an object the child would
    have to rebuild with knowledge of the DTO.
    """
    return json.dumps({"cmd": cmd, **fields}, ensure_ascii=False) + "\n"


def decode_event(line: str) -> WorkerEvent:
    """Parse one child event; anything unparseable is a protocol failure.

    A silently dropped line is the worst outcome here: the console would wait for a
    ``READY`` the child had already sent.
    """
    try:
        payload = json.loads(line)
    except ValueError as error:
        raise ChildProcessError(f"child sent a non-JSON line: {line[:120]!r}") from error
    if not isinstance(payload, dict) or "kind" not in payload:
        raise ChildProcessError(f"child sent a malformed event: {line[:120]!r}")
    try:
        kind = WorkerEventKind(payload["kind"])
    except ValueError as error:
        raise ChildProcessError(f"child sent unknown event kind {payload['kind']!r}") from error
    return WorkerEvent(
        kind,
        task_id=payload.get("task_id"),
        message=payload.get("message", ""),
        level=payload.get("level", "INFO"),
        success=payload.get("success"),
        payload=payload.get("payload"),
        vehicle=payload.get("vehicle"),
    )


class SubprocessMissionWorker:
    """The parent side: spawn the child, forward commands, relay its events.

    Method for method this is ``ThreadedMissionWorker`` seen from the other end of a
    pipe, which is what ``ServerController`` needs. Two behaviours come from having a
    process out there: reading runs on its own thread, because a blocking ``readline``
    is the only way to notice the child died without polling for it; and ``terminate``
    escalates to ``SIGKILL``, because a simulator client stuck inside an RPC never
    reaches an exit path.
    """

    def __init__(
        self,
        *,
        python: str,
        module: str,
        config_path: str,
        map_id: str,
        project_dir: str,
        log: Callable[[str, str], None] | None = None,
        popen: Callable[..., object] = subprocess.Popen,
        join_timeout_s: float = TERM_GRACE_S,
    ):
        self._python = python
        self._module = module
        self._config_path = config_path
        self._map_id = map_id
        self._project_dir = project_dir
        self._log_fn = log or (lambda message, level="INFO": None)
        self._popen = popen
        self._join_timeout_s = float(join_timeout_s)
        self._events: queue.Queue = queue.Queue()
        self._process: object | None = None
        self._reader: threading.Thread | None = None
        self._event_stream = None
        self._stdin = None

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        if self._process is not None:
            return
        env = dict(os.environ)
        # The child imports planner from the repository, not from site-packages.
        env["PYTHONPATH"] = self._project_dir + os.pathsep + env.get("PYTHONPATH", "")
        # A pipe of our own for the protocol: the child inherits the write end and is
        # told its number through the environment; stdout and stderr stay ordinary text.
        read_fd, write_fd = os.pipe()
        env[EVENT_FD_ENV] = str(write_fd)
        os.set_inheritable(write_fd, True)
        argv = [
            self._python, "-m", self._module,
            "--config", self._config_path,
            "--map", self._map_id,
        ]
        self._log_fn(f"Starting the mission child: {' '.join(argv)}")
        try:
            self._process = self._popen(
                argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                errors="replace",
                cwd=self._project_dir,
                env=env,
                bufsize=1,
                pass_fds=(write_fd,),
            )
        finally:
            os.close(write_fd)
        self._event_stream = os.fdopen(read_fd, "r", encoding="utf-8", errors="replace")
        self._stdin = self._process.stdin
        self._reader = threading.Thread(
            target=self._pump, name="mission-child-io", daemon=True
        )
        self._reader.start()

    def _pump(self) -> None:
        """Relay the child's protocol lines, then its raw output, into the event queue."""
        process = self._process
        assert process is not None
        for line in iter(self._event_stream.readline, ""):
            line = line.strip()
            if not line:
                continue
            try:
                self._events.put(decode_event(line))
            except ChildProcessError as error:
                self._events.put(WorkerEvent(WorkerEventKind.ERROR, message=str(error)))
        # Anything the child wrote to stdout or stderr bypassed the protocol on
        # purpose; forwarding it is what makes a crash diagnosable from the console
        # instead of only from a terminal.
        for stream in (process.stdout, process.stderr):
            for line in iter(stream.readline, ""):
                if line.strip():
                    self._events.put(
                        WorkerEvent(WorkerEventKind.LOG, message=line.rstrip()[:400])
                    )
        code = process.wait()
        self._events.put(
            WorkerEvent(WorkerEventKind.CLOSED, message=f"mission child exited with {code}")
        )

    def _send(self, line: str) -> None:
        if self._stdin is None:
            raise ChildProcessError("the mission child is not running")
        try:
            self._stdin.write(line)
            self._stdin.flush()
        except (BrokenPipeError, ValueError) as error:
            raise ChildProcessError(
                f"the mission child stopped accepting commands ({error}); read the log"
            ) from error

    # -- commands ----------------------------------------------------------
    def submit(self, task: TaskSubmission) -> None:
        self._send(encode_command(
            "task",
            task_id=task.task_id,
            instruction=task.instruction,
            vehicle=task.vehicle or "",
        ))

    def stop(self) -> None:
        if self._process is not None:
            self._send(encode_command("stop"))

    def shutdown(self, *, skip_landing: bool = False) -> None:
        if self._process is not None:
            self._send(encode_command("shutdown", skip_landing=bool(skip_landing)))

    def home(self, vehicle: str | None = None) -> None:
        self._send(encode_command("home", vehicle=vehicle or ""))

    def activate(self, vehicle: str) -> None:
        self._send(encode_command("activate", vehicle=vehicle))

    def set_view_mode(self, mode: str) -> None:
        self._send(encode_command("view", mode=mode))

    def set_weather(self, fields: dict[str, float]) -> None:
        self._send(encode_command("weather", fields=dict(fields)))

    def poll(self) -> WorkerEvent | None:
        try:
            return self._events.get_nowait()
        except queue.Empty:
            return None

    def terminate(self) -> None:
        process, reader = self._process, self._reader
        if process is None:
            return
        self._process = None
        self._stdin = None
        try:
            process.terminate()
        except Exception as error:
            self._log_fn(f"mission child was already gone: {error}", "WARNING")
        if reader is not None:
            reader.join(timeout=self._join_timeout_s)
            if reader.is_alive():
                self._log_fn(
                    f"mission child ignored SIGTERM for {self._join_timeout_s:g}s; "
                    "sending SIGKILL",
                    "WARNING",
                )
                process.kill()
        self._events.put(
            WorkerEvent(WorkerEventKind.CLOSED, message="mission child terminated")
        )
