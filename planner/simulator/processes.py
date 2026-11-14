"""Process primitives the simulator driver stands on.

Ported from ``../airsim-flight-console/scene_processes.py``, which runs them against
real UE builds. Nothing here knows about xpra argv, map catalogs or probes: it
answers questions about running processes and X displays, and it stops a process
tree only after proving the tree belongs to this project.

Two behaviours cost that project real debugging time and are easy to "simplify"
away, so they are named here as well as at the function:

* :func:`reclaim_stale_display` -- Xvfb deletes ``/tmp/.X<n>-lock`` only on a normal
  exit. Kill it and the next launch on that display fails without writing a byte to
  the engine log, because the child never started.
* :func:`display_released` -- ``xpra stop`` closes its TCP ports before the Xvfb it
  started lets go of the display. A restart that trusts the ports meets a live
  holder and refuses.
* :func:`terminate_verified_tree` -- a pid from a pidfile may have been recycled, so
  the identity read back must match the snapshot taken when the tree was ours.
"""

from __future__ import annotations

import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
from typing import Callable

from ..errors import SimulatorRestartError

LOOPBACK = "127.0.0.1"

#: Values carried over from the reference console's tuned defaults. The planner
#: config has no key for them, and inventing one for a number that is already
#: proven on real hardware is not worth a new setting nobody can tune.
POLL_INTERVAL_S = 0.2
KILL_WAIT_S = 5.0

WINDOW_PID_LINE = re.compile(r"^windows\.([^.]+)\.pid=(\d+)$", re.MULTILINE)


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


def tail(text: str | bytes | None, limit: int = 1500) -> str:
    """The end of a captured log, with a marker when the front was dropped."""
    if text is None:
        return ""
    if isinstance(text, bytes):
        text = text.decode("utf-8", "replace")
    text = text.strip()
    return text if len(text) <= limit else "..." + text[-limit:]
