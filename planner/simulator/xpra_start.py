"""Start and stop a simulator through ``xpra start --start-child``.

The ``script`` driver in :mod:`planner.simulator.supervisor` shells out to a
launch script that owns Xvfb, the engine and ``xpra shadow``. Families started
straight from a cooked engine binary have no such script: ``config/simulators.yaml``
carries the whole argv, and this module renders it, spawns it and waits for the
probes that family declares. Nothing here hardcodes a command; ``xpra``, the map
binary, the settings file and the ports all arrive from the family block.

The process lifecycle is a port of ``../airsim-flight-console``, which runs this
against real UE4 builds: ``scene_processes.py`` for identity, ownership, display
reclaim, window mapping and the stop ladder, ``environment_service.py`` for the
launch/wait shape. Two of its details are the whole reason it works, and both are
easy to "simplify" away:

* ``reclaim_stale_display`` — Xvfb deletes ``/tmp/.X<n>-lock`` only on a normal
  exit. Kill it and the next launch on that display fails with nothing in the
  engine log, because the child never started.
* ``window_ready`` — seamless xpra leaves a window unmapped until a client
  attaches, and UE4 pauses rendering for an unmapped window: the RPC port answers,
  ``simGetImages`` never returns, and the vehicle drifts down. Mapping and
  focusing the window through ``xpra control`` is the fix. ``-RenderOffscreen``
  hides the symptom and throws away the desktop picture as well.

One deliberate departure from the reference: its readiness check calls the weather
service, which imports ``airsim``. ``tests/test_architecture.py`` forbids loading a
simulator client in this process, so ``rpc`` here proves the port answers and the
holder is the map we asked for; the first real AirSim call happens in the UE4
runtime subprocess, where importing the client is allowed.
"""

from __future__ import annotations

import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import time
from types import SimpleNamespace
from typing import Callable

from ..config import MapCatalog, MapEntry, SimulatorSettings
from ..errors import SimulatorNotOwned, SimulatorRestartError
from .argv import argv_values, render_argv
from .supervisor import _tail

LogFn = Callable[[str, str], None]

LOOPBACK = "127.0.0.1"

#: Values carried over from the reference console's tuned defaults. The planner
#: config has no key for them, and inventing one for a number that is already
#: proven on real hardware is not worth a new setting nobody can tune.
POLL_INTERVAL_S = 0.2
KILL_WAIT_S = 5.0

_WINDOW_LINE = re.compile(r"^windows\.([^.]+)\.pid=(\d+)$", re.MULTILINE)


def port_alive(port: int, host: str = LOOPBACK, timeout_s: float = 0.3) -> bool:
    """True when something completes a TCP handshake, not merely a LISTEN entry.

    A wedged engine keeps its socket listening while refusing to serve.
    """
    with socket.socket() as connection:
        connection.settimeout(timeout_s)
        return connection.connect_ex((host, int(port))) == 0


class UnattributedListener(Exception):
    """A port is held and no readable owner exists, so nothing may be killed."""


def listener_pids(port: int) -> set[int]:
    """PIDs holding the listening socket, via ``ss`` like the reference does."""
    result = subprocess.run(
        ["ss", "-H", "-ltnp", f"sport = :{int(port)}"],
        capture_output=True,
        text=True,
        timeout=3,
    )
    pids = set(map(int, re.findall(r"pid=(\d+)", result.stdout or "")))
    if pids:
        return pids
    if port_alive(port):
        raise UnattributedListener(
            f"port {port} is in use but its owning process is not readable "
            "(another user, or ss missing); refusing to act on it"
        )
    return set()


def identity(pid: int) -> tuple | None:
    """(ppid, uid, exe, argv) for a PID, or None once it is gone.

    The tuple is the snapshot callers compare against before sending a signal, so
    a recycled PID is never killed by accident.
    """
    root = Path("/proc") / str(pid)
    try:
        fields = (root / "stat").read_text().rsplit(")", 1)[1].split()
        return (
            int(fields[19]),
            root.stat().st_uid,
            os.readlink(root / "exe"),
            (root / "cmdline")
            .read_bytes()
            .decode(errors="replace")
            .rstrip("\0")
            .split("\0"),
        )
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
        return None


def descendants(pid: int) -> list[tuple[int, tuple]]:
    """``pid`` and everything under it, through ``/proc/<pid>/task/*/children``."""
    found: list[tuple[int, tuple]] = []
    seen: set[int] = set()

    def walk(parent: int) -> None:
        if parent in seen:
            return
        seen.add(parent)
        try:
            tasks = Path(f"/proc/{parent}/task")
            children: set[int] = set()
            for task in tasks.iterdir():
                try:
                    children.update(
                        map(int, (task / "children").read_text().split())
                    )
                except (OSError, ValueError):
                    pass
            for child in sorted(children):
                walk(child)
            item = identity(parent)
            if item is not None:
                found.append((parent, item))
        except OSError:
            pass

    walk(pid)
    return found


def terminate_verified_tree(
    pid: int,
    expected: tuple,
    wait: float = KILL_WAIT_S,
    interval: float = POLL_INTERVAL_S,
    *,
    identity_reader: Callable[[int], tuple | None] = identity,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> list[int]:
    """SIGTERM the tree, let it clean up, SIGKILL only what is still identical.

    Xvfb removes its display lock and socket on a normal exit; SIGKILL leaves both
    behind and the display is unusable until someone reclaims it by hand.
    """
    if identity_reader(pid) != expected:
        return []
    captured = [
        (target, before)
        for target, before in descendants(pid)
        if before[1] == os.getuid()
    ]
    for target, before in captured:
        if identity_reader(target) == before:
            try:
                os.kill(target, signal.SIGTERM)
            except ProcessLookupError:
                pass
    alive = captured
    deadline = clock() + wait
    while alive and clock() < deadline:
        sleep(interval)
        alive = [(t, b) for t, b in alive if identity_reader(t) == b]
    for target, before in alive:
        if identity_reader(target) == before:
            try:
                os.kill(target, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return [target for target, _ in captured]


def display_files(display: str, lock_dir=Path("/tmp"), socket_dir=Path("/tmp/.X11-unix")):
    number = str(display)[1:]
    return lock_dir / f".X{number}-lock", socket_dir / f"X{number}"


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def reclaim_stale_display(display: str) -> str | None:
    """Delete a display lock whose Xvfb is gone. Raises if the display is live."""
    lock, sock = display_files(display)
    if not lock.exists():
        return None
    try:
        pid = int(lock.read_text().strip() or "0")
    except (OSError, ValueError) as error:
        raise SimulatorRestartError(
            f"cannot read the display lock {lock}: {error}"
        ) from error
    if pid <= 0:
        raise SimulatorRestartError(f"unreadable PID {pid} in the display lock {lock}")
    if pid_alive(pid):
        raise SimulatorRestartError(
            f"display {display} is still held by PID {pid}; stop it first or pick "
            "another simulator.display in config/base.yaml"
        )
    if lock.stat().st_uid != os.getuid():
        raise SimulatorRestartError(
            f"display lock {lock} belongs to UID {lock.stat().st_uid}; that user or "
            "root must remove it"
        )
    lock.unlink(missing_ok=True)
    sock.unlink(missing_ok=True)
    return str(lock)


def display_released(
    display: str,
    *,
    lock_dir: Path = Path("/tmp"),
    socket_dir: Path = Path("/tmp/.X11-unix"),
) -> bool:
    """True once no live process holds the X lock for this display.

    ``xpra stop`` closes its TCP ports before its Xvfb lets go of the display, so
    the ports going quiet is not the end of the teardown. Callers wait on this
    instead of guessing with a sleep.
    """
    lock, _ = display_files(display, lock_dir=lock_dir, socket_dir=socket_dir)
    if not lock.exists():
        return True
    try:
        pid = int(lock.read_text().strip() or "0")
    except (OSError, ValueError):
        return False
    return pid <= 0 or not pid_alive(pid)


def default_spawn(argv: list[str], **kwargs) -> subprocess.Popen:
    """Start ``argv`` detached and do not wait.

    ``xpra start --daemon=no`` stays in the foreground for the life of the
    session; waiting on it would block the HTTP process. stdout/stderr come from
    the caller so a launch that dies on the way in leaves a readable log.
    """
    kwargs.setdefault("stdin", subprocess.DEVNULL)
    kwargs.setdefault("stdout", subprocess.DEVNULL)
    kwargs.setdefault("stderr", subprocess.STDOUT)
    return subprocess.Popen(argv, start_new_session=True, **kwargs)


#: Stand-in for argv templates carrying no ``map.*`` placeholder, such as
#: ``stop_argv``. A real ``MapEntry`` validates its fields and cannot be empty.
_NO_MAP = SimpleNamespace(
    id="", map_name="", launch_map="", binary="", scene_file="", sim_config_path=""
)


class XpraStartSupervisor:
    """Drive one simulator family through a rendered ``xpra start`` argv."""

    def __init__(
        self,
        family,
        settings: SimulatorSettings,
        project_dir: str | Path,
        maps: MapCatalog,
        log: LogFn | None = None,
        *,
        spawn: Callable[..., object] = default_spawn,
        runner: Callable[..., object] = subprocess.run,
        port_checker: Callable[[int], bool] = port_alive,
        pid_finder: Callable[[int], set[int]] = listener_pids,
        identity_reader: Callable[[int], tuple | None] = identity,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self._family = family
        self._settings = settings
        self._project = Path(project_dir).resolve()
        self._maps = maps
        self._log_fn = log
        self._spawn = spawn
        self._runner = runner
        self._port = port_checker
        self._pid_finder = pid_finder
        self._identity = identity_reader
        self._clock = clock
        self._sleep = sleep
        self._runtime_dir = self._resolve(settings.runtime_dir)
        self._marker_file = self._resolve(settings.map_marker_file)
        self._process: object | None = None
        self._process_identity: tuple | None = None
        # ``-settings=`` is resolved by the engine child, whose working directory is
        # xpra's, not the repository's. Hand it an absolute path, and derive the
        # ownership fingerprint from the very same string so the two cannot drift.
        self._settings_path = str(self._resolve(family.settings_file)) if family.settings_file else ""
        # Families launched without a settings file (ProjectAirSim) get no marker at
        # all. Building "-settings=" anyway would leave a non-empty string that can
        # never appear in the engine argv, and every UE5 stack would read as foreign.
        self._settings_marker = f"-settings={self._settings_path}" if self._settings_path else ""
        for name in family.probes:
            if name not in SUPPORTED_PROBES:
                raise ValueError(
                    f"simulator '{family.id}' declares probe {name!r}; the "
                    f"xpra_start driver supports {sorted(SUPPORTED_PROBES)}"
                )
        if "rpc" in family.probes and "rpc" not in family.ports:
            raise ValueError(
                f"simulator '{family.id}' probes 'rpc' but declares no port named rpc"
            )

    # -- argv --------------------------------------------------------------
    def _base_values(self, entry) -> dict[str, object]:
        """Placeholder map for one launch, with the settings path made absolute.

        Both the child argv and the xpra argv read it from here. They used to build
        it separately, so a relative ``settings_file`` stayed relative inside
        ``--start-child`` while the ownership fingerprint carried the absolute form:
        UE4 could not find the file, aborted with signal 6, and xpra exited 0.
        """
        values = argv_values(
            entry=entry,
            simulator=self._family,
            settings=self._settings,
            runtime_dir=str(self._runtime_dir),
        )
        if self._settings_path:
            values["settings_file"] = self._settings_path
        return values

    def _values(self, entry=None) -> dict[str, object]:
        values = self._base_values(entry if entry is not None else _NO_MAP)
        if entry is not None:
            values["start_child"] = shlex.join(self.child_argv(entry))
        return values

    def child_argv(self, entry: MapEntry) -> list[str]:
        """The engine command line, before xpra wraps it in ``--start-child``."""
        return render_argv(list(self._family.child_argv), self._base_values(entry))

    def launch_argv(self, entry: MapEntry) -> list[str]:
        return render_argv(list(self._family.argv), self._values(entry))

    def stop_argv(self) -> list[str]:
        return render_argv(list(self._family.stop_argv), self._values())

    def launch_command(self, entry: MapEntry) -> str:
        return shlex.join(self.launch_argv(entry))

    # -- ownership ---------------------------------------------------------
    @property
    def ports(self) -> tuple[int, ...]:
        return tuple(self._family.ports.values())

    def _argv_of(self, pid: int) -> list[str]:
        item = self._identity(pid)
        if item is None:
            return []
        argv = list(item[3])
        # xpra rewrites its own argv with setproctitle into one display string.
        if len(argv) == 1 and argv[0].startswith("/usr/bin/xpra "):
            try:
                argv = shlex.split(argv[0])
            except ValueError:
                return []
        return argv

    def owned_map(self, pid: int) -> str | None:
        """Which catalogued map this PID runs, or None when it is not ours.

        Two fingerprints, matching the two processes that can hold a port: the
        engine binary itself (real path plus this project's settings file), and
        the xpra parent (our display, our socket directory, our binary inside
        ``--start-child``).
        """
        item = self._identity(pid)
        if item is None or item[1] != os.getuid():
            return None
        exe, argv = item[2], self._argv_of(pid)
        if not argv:
            return None
        for entry in self._maps.maps:
            if not entry.binary:
                continue
            real = os.path.realpath(entry.binary)
            # The binary path comes from this repository's own maps.yaml, so the
            # executable identity alone already proves the process is ours. The
            # settings marker tightens it for families that have one (AirSim) and
            # cannot apply to those that do not (ProjectAirSim has no -settings=).
            if exe == real and (not self._settings_marker or self._settings_marker in argv):
                return entry.id
            if (
                exe == os.path.realpath("/usr/bin/python3")
                and "/usr/bin/xpra" in argv
                and "start" in argv
                and self._settings.display in argv
                and f"--socket-dir={self._runtime_dir / 'sockets'}" in argv
                and any(
                    part.startswith("--start-child=") and entry.binary in part
                    for part in argv
                )
            ):
                return entry.id
        return None

    def holders(self) -> dict[int, tuple[str, tuple]]:
        """Every listener on our ports, attributed to a map. Raises otherwise.

        Validated in one pass before any signal is sent: half a stack torn down is
        worse than none stopped.
        """
        holders: dict[int, tuple[str, tuple]] = {}
        for port in self.ports:
            try:
                pids = self._pid_finder(port)
            except UnattributedListener as error:
                raise SimulatorNotOwned(str(error)) from error
            for pid in sorted(pids):
                key = self.owned_map(pid)
                if key is None:
                    argv = " ".join(self._argv_of(pid)) or "exited"
                    raise SimulatorNotOwned(
                        f"port {port} belongs to unmanaged PID {pid}: "
                        f"{_tail(argv, 200)}; nothing was stopped"
                    )
                holders[pid] = (key, self._identity(pid))
        return holders

    def running(self) -> bool:
        return bool(self.ports) and all(self._port(port) for port in self.ports)

    def preflight(self) -> None:
        """Refuse to touch a stack this project did not start."""
        if not self.running():
            return
        holders = self.holders()
        detail = "; ".join(
            f"{name}={pid}" for pid, (name, _) in holders.items()
        )
        self._log(f"Ports are held by our own stack ({detail}).", "DEBUG")

    # -- lifecycle ---------------------------------------------------------
    def start(self, entry: MapEntry) -> None:
        if self.running():
            current = self.holders()
            if all(name == entry.id for name, _ in current.values()):
                self._log(f"Map '{entry.id}' is already running.")
                return
            raise SimulatorRestartError(
                f"another map is live ({sorted({name for name, _ in current.values()})}); "
                f"switch to '{entry.id}' instead of starting over"
            )
        self.launch(entry)

    def launch(self, entry: MapEntry) -> None:
        self._check_binary(entry)
        self.preflight()
        reclaimed = reclaim_stale_display(self._settings.display)
        argv = self.launch_argv(entry)
        for folder in ("logs", "sockets", "sessions"):
            (self._runtime_dir / folder).mkdir(parents=True, exist_ok=True)
        launcher_log = self._runtime_dir / "logs" / f"xpra-start.{entry.id}.log"
        self._log(
            f"Launching map '{entry.id}' with xpra start on "
            f"{self._settings.display}"
            + (f" after reclaiming {reclaimed}" if reclaimed else "")
            + "."
        )
        try:
            with launcher_log.open("ab") as handle:
                handle.write(
                    f"\n--- {time.time()} {entry.id} ---\n".encode()
                )
                handle.flush()
                self._process = self._spawn(
                    argv,
                    cwd=str(self._project),
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                )
            self._process_identity = self._identity(_pid_of(self._process))
        except OSError as error:
            raise SimulatorRestartError(f"could not start {argv[0]}: {error}") from error
        try:
            self._wait_ready(entry, launcher_log)
        except Exception:
            # Never leave a half-started stack bound to the ports behind an error.
            self._cleanup_launch()
            raise
        self._marker_file.parent.mkdir(parents=True, exist_ok=True)
        self._marker_file.write_text(entry.id + "\n", encoding="utf-8")
        self._log(f"Map '{entry.id}' is ready.")

    def _check_binary(self, entry: MapEntry) -> None:
        binary = Path(entry.binary)
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise SimulatorRestartError(
                f"map '{entry.id}' binary is missing or not executable: {binary}"
            )

    def _wait_ready(self, entry: MapEntry, launcher_log: Path) -> None:
        hint = self._engine_log_hint(entry)
        if "ports" in self._family.probes:
            self._wait(
                lambda: all(self._port(p) for p in self.ports),
                "probes",
                launcher_log,
                description=f"map '{entry.id}' ports "
                + ", ".join(f"{n}={p}" for n, p in self._family.ports.items()),
                log_hint=hint,
            )
        if "rpc" in self._family.probes:
            self._wait(
                lambda: self._rpc_is_this_map(entry),
                "rpc",
                launcher_log,
                description=f"the rpc port answering as map '{entry.id}'",
                log_hint=hint,
            )
        if "window" in self._family.probes:
            self._wait(
                lambda: self.window_ready(entry),
                "window",
                launcher_log,
                description=f"the '{entry.id}' window mapped and focused",
                log_hint=hint,
            )

    def _engine_log_hint(self, entry: MapEntry) -> str:
        """Where the engine's own log will be, or an honest note that it has none.

        AirSim honours ``-abslog``. The ProjectAirSim builds were measured to write no
        log file and to print nothing on stdout, not even when they exit, so naming a
        path that can never appear throws away the one clue a timeout message carries.
        """
        for part in self.child_argv(entry):
            if part.startswith("-abslog="):
                return f"the engine's own log is {part.split('=', 1)[1]}"
        return "this family gives the engine no -abslog, so the xpra log is all there is"

    def _wait(self, condition, probe, launcher_log: Path, description: str,
              log_hint: str = "") -> None:
        """Poll until ready, giving up the moment the launcher itself is gone."""
        deadline = self._clock() + self._settings.launch_timeout_s
        while self._clock() < deadline:
            if self._process is not None and _poll(self._process) is not None:
                raise SimulatorRestartError(
                    f"the launch process exited with {getattr(self._process, 'returncode', '?')} "
                    f"while waiting for {description}; read {launcher_log}"
                )
            try:
                if condition():
                    return
            except UnattributedListener:
                raise
            self._sleep(POLL_INTERVAL_S)
        raise SimulatorRestartError(
            f"timed out after {self._settings.launch_timeout_s:g}s waiting for "
            f"{description}; {log_hint}; xpra said its piece in {launcher_log}"
        )

    def _rpc_is_this_map(self, entry: MapEntry) -> bool:
        """The RPC port answers and whoever holds it is running the map we asked for."""
        port = self._family.ports.get("rpc")
        if port is None or not self._port(port):
            return False
        holders = self.holders()
        return bool(holders) and all(name == entry.id for name, _ in holders.values())

    def _endpoint(self) -> str:
        number = str(self._settings.display)[1:]
        return f"socket://{self._runtime_dir / 'sessions' / number / 'socket'}"

    def _xpra(self, args: list[str]) -> subprocess.CompletedProcess:
        return self._runner(
            ["/usr/bin/xpra", *args, f"--socket-dir={self._runtime_dir / 'sockets'}",
             f"--socket-dirs={self._runtime_dir / 'sockets'}"],
            capture_output=True,
            text=True,
            timeout=8,
        )

    def window_ready(self, entry: MapEntry) -> bool:
        """Map and focus the engine window, exactly as the reference console does.

        Seamless xpra leaves a window unmapped until a client attaches; UE4 then
        pauses rendering, which is indistinguishable from a dead simulator to
        anything that asks for a camera frame.
        """
        try:
            info = self._xpra(["info", self._endpoint()])
        except (subprocess.TimeoutExpired, OSError):
            return False
        for window_id, pid in _WINDOW_LINE.findall(info.stdout or ""):
            if self.owned_map(int(pid)) != entry.id:
                continue
            for command in ("map", "focus"):
                try:
                    response = self._xpra(["control", self._endpoint(), command, window_id])
                except (subprocess.TimeoutExpired, OSError):
                    return False
                if getattr(response, "returncode", 1) != 0:
                    return False
            return True
        return False

    def stop(self) -> None:
        """Stop only what this project started, escalating from xpra to a tree kill."""
        holders = self.holders()
        if not holders:
            self._cleanup_launch()
            return
        self._log(f"Stopping the {self._family.id} stack ({len(holders)} listener(s)).")
        try:
            self._runner(
                self.stop_argv(),
                cwd=str(self._project),
                capture_output=True,
                text=True,
                timeout=self._settings.stop_timeout_s,
            )
        except (subprocess.TimeoutExpired, OSError) as error:
            self._log(f"xpra stop reported {type(error).__name__}; checking ports.", "WARNING")
        if self._wait_ports_down(self._settings.port_release_timeout_s):
            self._cleanup_launch()
            return
        # The polite path failed. Re-read and re-verify ownership, including the
        # identity snapshot, before anything is signalled.
        holders = self.holders()
        killed: list[int] = []
        for pid, (_, before) in holders.items():
            if before is None:
                continue
            killed.extend(
                terminate_verified_tree(
                    pid, before, KILL_WAIT_S, POLL_INTERVAL_S,
                    identity_reader=self._identity, sleep=self._sleep, clock=self._clock,
                )
            )
        if self._wait_ports_down(KILL_WAIT_S):
            self._cleanup_launch()
            return
        self.holders()
        raise SimulatorRestartError(
            f"the {self._family.id} stack still holds ports {list(self.ports)} after "
            f"xpra stop and a verified tree kill of {sorted(killed)}"
        )

    def _wait_ports_down(self, seconds: float) -> bool:
        deadline = self._clock() + seconds
        while self._clock() < deadline:
            if not any(self._port(port) for port in self.ports):
                return True
            self._sleep(POLL_INTERVAL_S)
        return not any(self._port(port) for port in self.ports)

    def _cleanup_launch(self) -> None:
        """Leave the machine ready for the next launch.

        Two parts, and only the first depends on who started the stack: reap the
        process this launch spawned, then wait for the display to come free. The
        wait cannot hide behind a ``_process is None`` return -- stopping a stack
        that a previous HTTP process started leaves no child of ours to reap at
        all, and that is exactly the case measured on the machine as
        ``restart`` refusing one second in with "display :104 is still held by
        PID".
        """
        if self._process is not None:
            pid = _pid_of(self._process)
            now = self._identity(pid)
            if (
                self._process_identity is not None
                and now is not None
                and now[:2] == self._process_identity[:2]
                and _poll(self._process) is None
            ):
                terminate_verified_tree(
                    pid, now, KILL_WAIT_S, POLL_INTERVAL_S,
                    identity_reader=self._identity, sleep=self._sleep, clock=self._clock,
                )
            try:
                self._process.wait(timeout=3)
            except (subprocess.TimeoutExpired, AttributeError):
                pass
            self._process = None
            self._process_identity = None
        # xpra's own Xvfb lets go of the display a moment after its ports close.
        # A display somebody else really owns still fails later in launch's
        # reclaim_stale_display, with that caller's message, so this stays silent
        # and bounded.
        self._wait_display_released(self._settings.port_release_timeout_s)

    def _wait_display_released(self, seconds: float) -> bool:
        deadline = self._clock() + seconds
        while self._clock() < deadline:
            if display_released(self._settings.display):
                return True
            self._sleep(POLL_INTERVAL_S)
        return display_released(self._settings.display)

    def restart(self, entry: MapEntry) -> None:
        if self.running():
            self.stop()
        self.launch(entry)

    # -- internals ---------------------------------------------------------
    def _resolve(self, path: str | Path) -> Path:
        candidate = Path(path)
        if candidate.is_absolute():
            return candidate
        return self._project / candidate

    def _log(self, message: str, level: str = "INFO") -> None:
        if self._log_fn is not None:
            self._log_fn(message, level)


#: Probes this driver can run. ``window`` is real: it maps and focuses the window
#: through xpra instead of merely looking for one.
SUPPORTED_PROBES = frozenset({"ports", "rpc", "window"})


def _pid_of(process) -> int:
    return int(getattr(process, "pid", 0) or 0)


def _poll(process):
    poll = getattr(process, "poll", None)
    return poll() if callable(poll) else None


def supervisor_for(
    family,
    settings: SimulatorSettings,
    project_dir: str | Path,
    maps: MapCatalog,
    log: LogFn | None = None,
    **kwargs,
):
    """Build the supervisor the family's driver calls for.

    ``script`` keeps the launch-script path used by ProjectAirSim; ``xpra_start``
    renders the argv the family declares. Both answers expose the same methods the
    controller uses, so no caller branches on a simulator name.
    """
    if family is None or family.driver == "script":
        from .supervisor import SimulatorSupervisor

        return SimulatorSupervisor(
            settings,
            project_dir,
            log=log,
            simulator=family,
            argv_template=family.argv if family is not None else None,
            **kwargs,
        )
    if family.driver == "xpra_start":
        return XpraStartSupervisor(family, settings, project_dir, maps, log=log, **kwargs)
    raise ValueError(
        f"simulator '{family.id}' declares driver {family.driver!r}, which "
        "planner/simulator has no implementation for"
    )
