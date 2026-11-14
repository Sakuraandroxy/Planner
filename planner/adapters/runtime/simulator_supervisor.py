"""Ownership-checked stop/relaunch of the ProjectAirSim + xpra stack.

ProjectAirSim cannot switch cooked UE maps inside a running process, so a map
change means stopping the whole stack and starting the target map again. The
launch script starts Xvfb, a windowed UE binary and ``xpra shadow`` and writes
pidfiles under ``simulator.runtime_dir``; the stop script only touches
processes whose pidfiles it wrote.

This module never kills an unowned process. Before a restart it verifies that
the running stack belongs to this project (pidfile + ``/proc/<pid>/cmdline``);
otherwise it reports the exact command the operator must run and aborts.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import subprocess
import time
from typing import Callable

from ...config import MapCatalog, MapEntry, SimulatorSettings
from ...errors import SimulatorNotOwned, SimulatorRestartError

LogFn = Callable[[str, str], None]


@dataclass(frozen=True)
class _OwnedPid:
    label: str
    pidfile: Path
    marker: str
    required: bool = True


def _tail(text: str | bytes | None, limit: int = 1500) -> str:
    if text is None:
        return ""
    if isinstance(text, bytes):
        text = text.decode("utf-8", "replace")
    text = text.strip()
    return text if len(text) <= limit else "..." + text[-limit:]


def _listen_ports() -> set[int]:
    ports: set[int] = set()
    for name in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            lines = Path(name).read_text(encoding="ascii").splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            parts = line.split()
            if len(parts) > 3 and parts[3] == "0A":  # TCP_LISTEN
                try:
                    ports.add(int(parts[1].rsplit(":", 1)[1], 16))
                except (IndexError, ValueError):
                    continue
    return ports


def port_listening(port: int) -> bool:
    """True when something listens on the TCP port (no connection is made)."""
    return int(port) in _listen_ports()


def read_proc_cmdline(pid: int) -> str | None:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return None
    if not raw:
        return None
    return raw.replace(b"\0", b" ").decode("utf-8", "replace")


def read_map_marker(path: str | Path, catalog: MapCatalog) -> str | None:
    """Return the marker's map id when it names a catalog entry."""
    try:
        value = Path(path).read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return value if catalog.entry(value) is not None else None


class SimulatorSupervisor:
    """Stop and relaunch the simulator stack for a catalog map entry."""

    def __init__(
        self,
        settings: SimulatorSettings,
        project_dir: str | Path,
        log: LogFn | None = None,
        *,
        runner: Callable[..., object] = subprocess.run,
        port_checker: Callable[[int], bool] = port_listening,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        cmdline_reader: Callable[[int], str | None] = read_proc_cmdline,
    ):
        self._settings = settings
        self._project_dir = Path(project_dir).resolve()
        self._log_fn = log
        self._runner = runner
        self._port_checker = port_checker
        self._clock = clock
        self._sleep = sleep
        self._cmdline_reader = cmdline_reader
        self._runtime_dir = self._resolve(settings.runtime_dir)
        self._marker_file = self._resolve(settings.map_marker_file)
        self._display_name = settings.display[1:]

    # -- public API --------------------------------------------------------
    def running(self) -> bool:
        """True when the xpra, topic and service ports are all listening."""
        return all(
            self._port_checker(port)
            for port in (
                self._settings.xpra_port,
                self._settings.topics_port,
                self._settings.services_port,
            )
        )

    def launch_command(self, entry: MapEntry) -> str:
        """The exact launch-script invocation, for operator-facing logs."""
        return (
            f"{self._settings.launch_script} {entry.launch_map} "
            f"{self._settings.display} {self._settings.xpra_port} "
            f"{self._settings.topics_port} {self._settings.services_port}"
        )

    def preflight(self) -> None:
        """Refuse to restart a stack that this project does not own."""
        for script in (
            self._resolve(self._settings.launch_script),
            self._resolve(self._settings.stop_script),
        ):
            if not script.is_file():
                raise SimulatorRestartError(f"script not found: {script}")
        missing: list[str] = []
        for owned in self._required_processes():
            pid = self._read_pid(owned.pidfile)
            if pid is None:
                if owned.required:
                    missing.append(f"{owned.label} ({owned.pidfile})")
                continue
            cmdline = self._cmdline_reader(pid)
            if cmdline is None or owned.marker not in cmdline:
                if owned.required:
                    missing.append(
                        f"{owned.label} PID {pid} (pidfile {owned.pidfile})"
                    )
        if missing:
            raise SimulatorNotOwned(
                "the simulator stack was not started by "
                f"{self._settings.launch_script}; refusing to stop it. "
                "Unverified: " + ", ".join(missing) + ". Start it with "
                f"{self._settings.launch_script} <map> {self._settings.display} "
                f"{self._settings.xpra_port} {self._settings.topics_port} "
                f"{self._settings.services_port} (or stop it yourself), "
                "then retry the map switch."
            )

    def stop(self, *, sim_only: bool = False) -> None:
        """Run the stop script, then wait until ports and the X lock are free.

        With ``sim_only`` only the UE process is stopped; Xvfb and the xpra
        shadow keep running so a map switch does not reconnect the console.
        """
        if sim_only:
            self._log(
                "Stopping only the simulator; keeping Xvfb and xpra.", "INFO"
            )
            args = [self._settings.display, "--sim-only"]
        else:
            self._log("Stopping the ProjectAirSim + xpra stack.", "INFO")
            args = [self._settings.display]
        output = self._run_script(
            self._resolve(self._settings.stop_script),
            args,
            self._settings.stop_timeout_s,
        )
        expected_pids = {
            self._read_pid(owned.pidfile)
            for owned in self._required_processes()
        }
        deadline = self._clock() + self._settings.port_release_timeout_s
        while True:
            if sim_only:
                ports = [
                    self._settings.topics_port,
                    self._settings.services_port,
                ]
                lock_gone = True
                pidfiles_gone = self._read_pid(self._sim_pidfile()) is None
            else:
                ports = [
                    self._settings.xpra_port,
                    self._settings.topics_port,
                    self._settings.services_port,
                ]
                lock = Path(f"/tmp/.X{self._display_name}-lock")
                lock_gone = not lock.exists()
                pidfiles_gone = all(
                    self._read_pid(owned.pidfile) is None
                    for owned in self._required_processes()
                    if owned.required
                )
            free = all(not self._port_checker(port) for port in ports)
            if free and lock_gone and pidfiles_gone:
                self._log(
                    "Simulator stopped; Xvfb and xpra kept."
                    if sim_only
                    else "Simulator stack stopped.",
                    "INFO",
                )
                return
            if self._clock() >= deadline:
                raise SimulatorRestartError(
                    "simulator stack did not stop within "
                    f"{self._settings.port_release_timeout_s:g}s: "
                    f"ports_free={free} x_lock_gone={lock_gone} "
                    f"pidfiles_gone={pidfiles_gone} "
                    f"(stale PIDs survived: {sorted(p for p in expected_pids if p)})"
                    + (f"\n{output}" if output else "")
                )
            self._sleep(0.2)

    def launch(self, entry: MapEntry, *, sim_only: bool = False) -> None:
        """Start the map through the launch script and verify readiness.

        With ``sim_only`` the script restarts only the UE binary on the live
        Xvfb; the xpra shadow follows the new window on the same display.
        """
        target = (
            "restarting the simulator; keeping Xvfb and xpra"
            if sim_only
            else f"on {self._settings.display}"
        )
        self._log(
            f"Launching map '{entry.id}' ({entry.launch_map}) {target}.",
            "INFO",
        )
        args = [
            entry.launch_map,
            self._settings.display,
            str(self._settings.xpra_port),
            str(self._settings.topics_port),
            str(self._settings.services_port),
        ]
        if sim_only:
            args.append("--sim-only")
        output = self._run_script(
            self._resolve(self._settings.launch_script),
            args,
            self._settings.launch_timeout_s,
        )
        ports = (
            (self._settings.topics_port, self._settings.services_port)
            if sim_only
            else (
                self._settings.xpra_port,
                self._settings.topics_port,
                self._settings.services_port,
            )
        )
        deadline = self._clock() + self._settings.port_release_timeout_s
        while not all(self._port_checker(port) for port in ports):
            if self._clock() >= deadline:
                raise SimulatorRestartError(
                    f"map '{entry.id}' did not open ports {ports} within "
                    f"{self._settings.port_release_timeout_s:g}s"
                    + (f"\n{output}" if output else "")
                )
            self._sleep(0.5)
        # The scripts already waited for the window (and xpra on a full
        # launch); the session retries the connect probe until the server
        # answers, so only a short settle is needed here.
        self._sleep(0.5)
        self._marker_file.parent.mkdir(parents=True, exist_ok=True)
        self._marker_file.write_text(entry.id + "\n", encoding="utf-8")
        self._log(f"Map '{entry.id}' is ready.", "INFO")

    def start(self, entry: MapEntry) -> None:
        """Launch the map when no stack is running; never stops anything."""
        if self.running():
            raise SimulatorRestartError(
                "a simulator stack is already running; use a map switch"
            )
        self.launch(entry)

    def restart(self, entry: MapEntry) -> None:
        """One stop/launch attempt; raises on any failure, writes no marker.

        When Xvfb and xpra are still alive only the UE process is restarted,
        which keeps the console's xpra connection intact.
        """
        self.preflight()
        keep_view = self._view_stack_alive()
        self._log(
            "Xvfb and xpra are up; restarting only the simulator."
            if keep_view
            else "Xvfb or xpra is down; restarting the whole stack.",
            "INFO",
        )
        self.stop(sim_only=keep_view)
        self.launch(entry, sim_only=keep_view)

    # -- internals ---------------------------------------------------------
    def _resolve(self, path: str | Path) -> Path:
        candidate = Path(path)
        if candidate.is_absolute():
            return candidate
        return self._project_dir / candidate

    def _sim_pidfile(self) -> Path:
        return self._runtime_dir / f"sim{self._display_name}.pid"

    def _xvfb_pidfile(self) -> Path:
        return self._runtime_dir / f"xvfb{self._display_name}.pid"

    def _view_stack_alive(self) -> bool:
        """True when Xvfb and xpra are up, so only the UE process must restart."""
        if not self._port_checker(self._settings.xpra_port):
            return False
        pid = self._read_pid(self._xvfb_pidfile())
        if pid is None:
            return False
        cmdline = self._cmdline_reader(pid) or ""
        return f"Xvfb {self._settings.display}" in cmdline

    def _required_processes(self) -> tuple[_OwnedPid, ...]:
        runtime = self._runtime_dir
        name = self._display_name
        return (
            _OwnedPid(
                "simulator",
                runtime / f"sim{name}.pid",
                str(self._project_dir / "sim"),
            ),
            _OwnedPid(
                "Xvfb",
                runtime / f"xvfb{name}.pid",
                f"Xvfb {self._settings.display}",
            ),
            _OwnedPid(
                "shadow",
                runtime / f"shadow{name}.pid",
                "xpra",
                required=False,
            ),
        )

    def _read_pid(self, pidfile: Path) -> int | None:
        try:
            value = pidfile.read_text(encoding="ascii").strip()
        except OSError:
            return None
        if not value.isdigit():
            return None
        pid = int(value)
        if pid <= 0 or self._cmdline_reader(pid) is None:
            return None
        return pid

    def _run_script(self, script: Path, args: list[str], timeout_s: float) -> str:
        argv = [str(script), *args]
        env = dict(os.environ)
        env["PROJECTAIRSIM_XPRA_RUNTIME"] = str(self._runtime_dir)
        try:
            completed = self._runner(
                argv,
                cwd=str(self._project_dir),
                capture_output=True,
                text=True,
                timeout=timeout_s,
                env=env,
            )
        except subprocess.TimeoutExpired as error:
            raise SimulatorRestartError(
                f"{script.name} timed out after {timeout_s:g}s"
            ) from error
        except OSError as error:
            raise SimulatorRestartError(
                f"could not run {script}: {error}"
            ) from error
        code = getattr(completed, "returncode", 1)
        output = "\n".join(
            part
            for part in (
                _tail(getattr(completed, "stdout", "")),
                _tail(getattr(completed, "stderr", "")),
            )
            if part
        )
        if code != 0:
            raise SimulatorRestartError(
                f"{script.name} exited with {code}"
                + (f"\n{output}" if output else "")
            )
        return output

    def _log(self, message: str, level: str = "INFO") -> None:
        if self._log_fn is not None:
            self._log_fn(message, level)
