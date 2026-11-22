"""Tests for the ``xpra_start`` process driver.

Nothing is launched here. The runner, the port checks, the PID lookup and the
``/proc`` reader are faked, so these assert the argv that would reach xpra, and
every condition under which the driver refuses to act or escalates.

The ownership rules are the point of this file. They are copied from
``../airsim-flight-console/scene_processes.py``, where they are what stands between
a map switch and ``SIGKILL``ing somebody else's process.
"""

import atexit
import os
import shutil
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from planner.config import MapCatalog, MapEntry, MapScene, SimulatorSettings
from planner.domain.engine_family import SimulatorFamily, parse_family_catalog
from planner.errors import SimulatorNotOwned, SimulatorRestartError
from planner.simulator.processes import (
    UnattributedListener,
    display_files,
)
from planner.simulator.xpra_start import (
    SUPPORTED_PROBES,
    XpraStartSupervisor,
)

DISPLAY = ":199"
RPC_PORT = 41451
XPRA_PORT = 20010
REPO = Path(__file__).resolve().parents[1]

#: The driver refuses to launch a map whose binary is absent, so these point at real
#: executable stubs instead of a hardcoded install path: the same test then passes on
#: a machine that has no AirSim binaries at all.
STUBS = Path(tempfile.mkdtemp(prefix="xpra-start-driver-"))
atexit.register(shutil.rmtree, STUBS, True)


def stub_binary(name):
    path = STUBS/name
    path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    path.chmod(0o755)
    return str(path)


SETTINGS = str(STUBS/"settings.json")
BLOCKS = stub_binary("Blocks")
CAMPUS = stub_binary("Campus")


def _family(**overrides):
    values = dict(
        id="airsim",
        label="AirSim (UE4)",
        driver="xpra_start",
        runtime="planner.simulators.airsim",
        settings_file=SETTINGS,
        ports={"xpra": XPRA_PORT, "rpc": RPC_PORT},
        probes=("ports", "rpc", "window"),
        child_argv=(
            "{map.binary}",
            "{map.map_name}",
            "-settings={settings_file}",
            "-abslog={abslog}",
            "-windowed",
            "-ResX={res_x}",
            "-ResY={res_y}",
        ),
        argv=(
            "/usr/bin/xpra",
            "start",
            "{display}",
            "--start-child={start_child}",
            "--bind-tcp=0.0.0.0:{ports.xpra}",
            "--log-dir={runtime_dir}/logs",
            "--pidfile={runtime_dir}/xpra.pid",
            "--xvfb=Xvfb -screen 0 {res_x}x{res_y}x24",
            "--socket-dir={runtime_dir}/sockets",
        ),
        stop_argv=("/usr/bin/xpra", "stop", "socket://{runtime_dir}/sessions/{display_number}/socket"),
    )
    values.update(overrides)
    return SimulatorFamily(**values)


def _maps():
    return MapCatalog(
        default_map="blocks",
        maps=(
            MapEntry(id="blocks", label="Blocks", simulator="airsim",
                     binary=BLOCKS, map_name="Blocks"),
            MapEntry(id="campus", label="Campus", simulator="airsim",
                     binary=CAMPUS, map_name="Campus"),
            MapEntry(id="cityenviron", label="City", simulator="projectairsim",
                     launch_map="cityenviron", scene=MapScene(file="scene_city_dual.jsonc", config_path="config/scenes/city")),
        ),
    )


def _entry(map_id="blocks"):
    return _maps().entry(map_id)


def _settings(tmp_path, **overrides):
    values = dict(
        display=DISPLAY,
        runtime_dir=str(tmp_path / "output/xpra"),
        map_marker_file=str(tmp_path / "output/xpra/current_map"),
        launch_timeout_s=5.0,
        stop_timeout_s=5.0,
        port_release_timeout_s=0.5,
    )
    values.update(overrides)
    return SimulatorSettings(**values)


def _identity(pid, exe, argv, uid=None):
    """(ppid, uid, exe, argv) in the shape ``identity()`` returns."""
    return (1, os.getuid() if uid is None else uid, exe, list(argv))


class Harness:
    """Driver wired to fakes: which ports answer, which PIDs exist, what they run."""

    def __init__(self, tmp_path, family=None, *, alive=(), holders=None, windows=None,
                 uid=None, strict_pids=False, maps=None):
        self.family = family or _family()
        self.now = 0.0
        self.alive = set(alive)                 # ports accepting a connection
        # port -> {pid: (map_id, exe, argv)}
        self.bindings = {int(port): dict(value) for port, value in (holders or {}).items()}
        self.uid = os.getuid() if uid is None else uid
        self.windows = dict(windows or {})      # window id -> pid
        self.spawned = []
        self.calls = []                         # every xpra/subprocess invocation
        self.exit_code = None                   # what the spawned process returns

        def spawn(argv, **kwargs):
            self.spawned.append((argv, kwargs))
            harness_listening = self

            class Fake:
                pid = 4242

                @property
                def returncode(self):
                    return harness_listening.exit_code

                def poll(self):
                    return harness_listening.exit_code

                def wait(self, timeout=None):
                    return self.returncode

            return Fake()

        def run(argv, **kwargs):
            """Answers ``xpra stop``, ``xpra info`` and ``xpra control``."""
            self.calls.append(list(argv))
            verb = argv[1] if len(argv) > 1 else ""
            if verb == "info":
                body = "".join(
                    f"windows.{wid}.pid={pid}\n" for wid, pid in self.windows.items()
                )
                return SimpleNamespace(returncode=0, stdout=body, stderr="")
            if self.stop_frees_ports and verb == "stop":
                self.alive.clear()
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        self.stop_frees_ports = True
        self.strict_pids = strict_pids

        def pid_finder(port):
            owners = self.bindings.get(int(port), {})
            if not owners:
                if int(port) in self.alive and self.strict_pids:
                    raise UnattributedListener(f"port {port} unreadable owner")
                return set()
            return set(owners)

        def identity_reader(pid):
            for owners in self.bindings.values():
                if pid in owners:
                    _map_id, exe, argv = owners[pid]
                    return _identity(pid, exe, self.expand(argv), uid=self.uid)
            return None

        self.driver = XpraStartSupervisor(
            self.family,
            _settings(tmp_path),
            tmp_path,
            maps if maps is not None else _maps(),
            spawn=spawn,
            runner=run,
            port_checker=lambda port: int(port) in self.alive,
            pid_finder=pid_finder,
            identity_reader=identity_reader,
            clock=self.clock,
            sleep=self.sleep,
        )
        self.harness = self

    def expand(self, argv):
        """Fill the placeholders a real engine cmdline would carry."""
        return [
            part.replace("@SETTINGS@", SETTINGS).replace("@BLOCKS@", BLOCKS)
            .replace("@RUNTIME@", str(self.driver._runtime_dir))
            .replace("@DISPLAY@", DISPLAY)
            for part in argv
        ]

    def bring_up(self, map_id="blocks"):
        """Put the fakes into the state a running stack leaves behind."""
        owners = {77: (map_id, os.path.realpath(BLOCKS if map_id == "blocks" else
                                               _entry(map_id).binary),
                       [_entry(map_id).binary, _entry(map_id).map_name,
                        f"-settings={SETTINGS}"])}
        self.bindings[XPRA_PORT] = dict(owners)
        self.bindings[RPC_PORT] = dict(owners)
        self.alive.update({XPRA_PORT, RPC_PORT})
        self.windows = {"9": 77}

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def ours_engine(pid=77):
    """A holder table with the engine child attributed to ``blocks`` on both ports."""
    exe = os.path.realpath(BLOCKS)
    argv = [BLOCKS, "Blocks", f"-settings={SETTINGS}", "-windowed"]
    return {pid: ("blocks", exe, argv)}


# -- argv rendering --------------------------------------------------------
def test_child_argv_fills_every_placeholder(tmp_path):
    driver = Harness(tmp_path).driver
    assert driver.child_argv(_entry()) == [
        BLOCKS,
        "Blocks",
        f"-settings={SETTINGS}",
        f"-abslog={driver._runtime_dir / 'logs' / 'blocks.log'}",
        "-windowed",
        "-ResX=1280",
        "-ResY=720",
    ]


def test_start_child_survives_quoting_as_one_argument(tmp_path):
    argv = Harness(tmp_path).driver.launch_argv(_entry())
    child = [part for part in argv if part.startswith("--start-child=")]
    assert len(child) == 1, "xpra takes --start-child as a single argument"
    assert BLOCKS in child[0] and f"-settings={SETTINGS}" in child[0]


def test_xvfb_geometry_follows_resolution(tmp_path):
    argv = Harness(tmp_path).driver.launch_argv(_entry())
    relevant = next(p for p in argv if p.startswith("--xvfb="))
    assert "1280x720x24" in relevant


def test_unresolved_placeholder_is_an_error_not_a_literal(tmp_path):
    """A stray ``{binary}`` would reach xpra and surface as a session with no child."""
    broken = _family(child_argv=("{binary}", "{map.map_name}"))
    with pytest.raises(ValueError, match=r"binary"):
        Harness(tmp_path, family=broken).driver.child_argv(_entry())


def test_empty_value_is_rejected(tmp_path):
    loose = _family(settings_file="")
    with pytest.raises(ValueError, match="resolved to nothing"):
        Harness(tmp_path, family=loose).driver.child_argv(_entry())


def test_half_a_binary_pair_is_refused_by_the_catalog():
    with pytest.raises(ValueError, match="binary and map_name"):
        MapEntry(id="x", label="X", simulator="airsim", binary="", map_name="Blocks")


def test_stop_argv_needs_no_map(tmp_path):
    argv = Harness(tmp_path).driver.stop_argv()
    assert argv[:2] == ["/usr/bin/xpra", "stop"]
    assert argv[2].endswith("/sessions/199/socket")
    assert "{" not in "".join(argv)


def test_relative_settings_path_is_absolutised(tmp_path):
    """The engine child resolves ``-settings=`` from xpra's cwd, not the repo's.

    A relative path made UE4 abort with signal 6 while xpra still exited 0, so the
    failure looked like a hang. The fingerprint and the argv must agree too, or
    ownership checks stop matching the stack they describe.
    """
    loose = _family(settings_file="config/airsim_settings.json")
    driver = Harness(tmp_path, family=loose).driver
    child = driver.child_argv(_entry())
    assert driver._settings_path == str(Path(driver._settings_path))
    assert f"-settings={driver._settings_path}" in child
    assert driver._settings_marker == f"-settings={driver._settings_path}"


def test_shipped_template_renders(tmp_path):
    """The real config/simulators.yaml must render; it once referenced {binary}."""
    catalog = parse_family_catalog(
        yaml.safe_load((REPO / "config/simulators.yaml").read_text(encoding="utf-8"))
    )
    family = catalog.require("airsim")
    driver = XpraStartSupervisor(
        family, _settings(tmp_path), tmp_path, load_maps(),
        spawn=lambda argv, **k: SimpleNamespace(pid=1, poll=lambda: None, wait=lambda **_: 0),
        runner=lambda argv, **k: SimpleNamespace(returncode=0, stdout="", stderr=""),
        port_checker=lambda port: True,
    )
    argv = driver.launch_argv(_entry())
    assert not [part for part in argv if "{" in part], argv
    assert family.probes == ("window", "ports", "rpc")
    assert "-RenderOffscreen" not in driver.child_argv(_entry())


def load_maps():
    from planner.config import load_map_catalog

    return load_map_catalog(REPO / "config/maps.yaml")


def test_shipped_templates_render_for_every_family(tmp_path):
    """Both families in the shipped YAML must render, not just AirSim.

    ProjectAirSim declares no settings_file, so a template that reaches for
    {settings_file} would abort on the machine rather than in review. The engine
    argv is also where the map's own binary and map name have to land.
    """
    catalog = parse_family_catalog(
        yaml.safe_load((REPO / "config/simulators.yaml").read_text(encoding="utf-8"))
    )
    maps = load_maps()
    for family_id in ("airsim", "projectairsim"):
        family = catalog.require(family_id)
        entry = next(item for item in maps.maps if item.simulator == family_id)
        driver = XpraStartSupervisor(
            family, _settings(tmp_path), tmp_path, maps,
            spawn=lambda argv, **k: SimpleNamespace(pid=1, poll=lambda: None, wait=lambda **_: 0),
            runner=lambda argv, **k: SimpleNamespace(returncode=0, stdout="", stderr=""),
            port_checker=lambda port: True,
        )
        child = driver.child_argv(entry)
        assert child[0] == entry.binary, child
        assert child[1] == entry.map_name, child
        argv = driver.launch_argv(entry)
        assert not [part for part in argv if "{" in part], argv
        assert not [part for part in driver.stop_argv() if "{" in part]
        assert driver._settings_marker == ("" if family_id == "projectairsim"
                                          else f"-settings={driver._settings_path}")


def test_timeout_hint_names_the_log_the_family_actually_writes(tmp_path):
    """A hint to a file that can never appear is worse than no hint at all.

    AirSim honours -abslog; the ProjectAirSim builds were measured to write no log
    file. Both statements have to come out of the same function that builds the
    argv, or the message drifts away from the flags the process really got.
    """
    driver = Harness(tmp_path).driver
    assert driver._engine_log_hint(_entry()).endswith("logs/blocks.log")

    ue5 = Harness(tmp_path, family=_ue5_family(), maps=_ue5_maps())
    hint = ue5.driver._engine_log_hint(_ue5_maps().entry("cityenviron"))
    assert "no -abslog" in hint, hint
    assert ".log" not in hint, hint


# -- ownership -------------------------------------------------------------
def test_engine_child_is_ours_by_exe_and_settings_marker(tmp_path):
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    assert harness.driver.owned_map(77) == "blocks"


CITY = "/roxy/vln/projectairsim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping"
TOPICS_PORT = 20991
SERVICES_PORT = 20992


def _ue5_family():
    """The ProjectAirSim shape: no settings file, socket ports held by the engine."""
    return _family(
        id="projectairsim",
        label="ProjectAirSim (UE5)",
        runtime="planner.simulators.projectairsim",
        settings_file="",
        ports={"xpra": XPRA_PORT, "topics": TOPICS_PORT, "services": SERVICES_PORT},
        probes=("window", "ports"),
        child_argv=(
            "{map.binary}", "{map.map_name}", "-windowed", "-ResX={res_x}",
            "-ResY={res_y}", "-nosound", "-log",
            "-topicsport={ports.topics}", "-servicesport={ports.services}",
        ),
    )


def _ue5_maps():
    return MapCatalog(
        default_map="cityenviron",
        maps=(
            MapEntry(id="cityenviron", label="City", simulator="projectairsim",
                     binary=CITY, map_name="CityEnviron",
                     scene=MapScene(file="scene_city_dual.jsonc", config_path="config/scenes/city")),
        ),
    )


def test_a_settings_free_stack_is_ours_by_binary_alone(tmp_path):
    """A family without a settings file must not be forced to carry the marker.

    The fingerprint used to be built unconditionally, so an empty ``settings_file``
    still produced the non-empty string ``"-settings="``, which can never appear in
    the engine argv. Every ProjectAirSim stack therefore read as foreign, and the
    driver refused to stop processes it had started itself.
    """
    harness = Harness(
        tmp_path,
        family=_ue5_family(),
        maps=_ue5_maps(),
        holders={TOPICS_PORT: {77: ("cityenviron", os.path.realpath(CITY),
                                    [CITY, "CityEnviron", "-windowed", "-log"])}},
        alive={TOPICS_PORT},
    )
    assert harness.driver._settings_marker == ""
    assert harness.driver.owned_map(77) == "cityenviron"


def test_xpra_parent_is_ours_by_display_and_socket_dir(tmp_path):
    runtime = Harness(tmp_path).driver._runtime_dir
    argv = ["/usr/bin/xpra", "start", DISPLAY, f"--socket-dir={runtime / 'sockets'}",
            f"--start-child={BLOCKS} Blocks"]
    harness = Harness(
        tmp_path,
        holders={XPRA_PORT: {88: ("blocks", os.path.realpath("/usr/bin/python3"), argv)}},
        alive={XPRA_PORT},
    )
    assert harness.driver.owned_map(88) == "blocks"


def test_setproctitle_collapsed_argv_is_still_read(tmp_path):
    """xpra replaces its argv with one string; shlex has to put it back."""
    runtime = Harness(tmp_path).driver._runtime_dir
    line = (f"/usr/bin/xpra start {DISPLAY} --socket-dir={runtime / 'sockets'} "
            f"--start-child='{BLOCKS} Blocks'")
    harness = Harness(
        tmp_path,
        holders={XPRA_PORT: {89: ("blocks", os.path.realpath("/usr/bin/python3"), [line])}},
        alive={XPRA_PORT},
    )
    assert harness.driver.owned_map(89) == "blocks"


def test_foreign_listener_blocks_a_stop(tmp_path):
    foreign = {99: ("?", "/usr/bin/something_else", ["--port", str(RPC_PORT)])}
    harness = Harness(tmp_path, holders={RPC_PORT: foreign}, alive={RPC_PORT})
    with pytest.raises(SimulatorNotOwned, match="unmanaged PID 99"):
        harness.driver.stop()
    assert harness.calls == [], "no command may run against a stack we do not own"


def test_unattributable_listener_blocks_a_stop(tmp_path):
    harness = Harness(tmp_path, alive={RPC_PORT}, strict_pids=True)  # no readable owner
    with pytest.raises(SimulatorNotOwned, match="unreadable owner"):
        harness.driver.stop()


def test_process_owned_by_another_user_is_not_ours(tmp_path):
    other = {91: ("blocks", os.path.realpath(BLOCKS),
                  [BLOCKS, "Blocks", f"-settings={SETTINGS}"])}
    harness = Harness(tmp_path, holders={RPC_PORT: other}, alive={RPC_PORT}, uid=os.getuid() + 1)
    with pytest.raises(SimulatorNotOwned):
        harness.driver.stop()


def test_a_different_display_is_not_ours(tmp_path):
    """Same binary and settings, started by hand on another display."""
    harness = Harness(tmp_path, alive={XPRA_PORT})
    argv = ["/usr/bin/xpra", "start", ":250",
            f"--socket-dir={harness.driver._runtime_dir / 'sockets'}",
            f"--start-child={BLOCKS} Blocks"]
    harness.bindings[XPRA_PORT] = {
        95: ("blocks", os.path.realpath("/usr/bin/python3"), argv)
    }
    with pytest.raises(SimulatorNotOwned):
        harness.driver.stop()


# -- readiness -------------------------------------------------------------
def test_window_probe_maps_and_focuses(tmp_path):
    """The window must be mapped, or UE4 pauses rendering and capture never returns."""
    harness = Harness(
        tmp_path,
        holders={RPC_PORT: ours_engine()},
        alive={RPC_PORT},
        windows={"4194356": 77},
    )
    assert harness.driver.window_ready(_entry())
    mapped = [c for c in harness.calls if c[1] == "control" and c[3] == "map"]
    focused = [c for c in harness.calls if c[1] == "control" and c[3] == "focus"]
    assert [c[4] for c in mapped] == ["4194356"]
    assert [c[4] for c in focused] == ["4194356"]


def test_window_probe_ignores_a_window_that_is_not_ours(tmp_path):
    harness = Harness(tmp_path, windows={"1": 77})  # no holder table, so PID 77 is unknown
    assert not harness.driver.window_ready(_entry())


def test_launch_waits_for_ports_rpc_and_window(tmp_path):
    harness = Harness(tmp_path)

    def spawn(argv, **kwargs):
        harness.spawned.append((argv, kwargs))
        harness.bring_up()
        return _ok_process()

    harness.driver._spawn = spawn
    harness.driver.launch(_entry())
    assert harness.driver.running()


def test_launch_does_not_report_ready_on_an_unmapped_window(tmp_path):
    harness = Harness(tmp_path)

    def spawn(argv, **kwargs):
        harness.bring_up()
        harness.windows = {}          # xpra has the session but no mapped window yet
        return _ok_process()

    harness.driver._spawn = spawn
    with pytest.raises(SimulatorRestartError, match="window"):
        harness.driver.launch(_entry())


def test_launch_fails_when_the_rpc_holder_is_another_map(tmp_path):
    harness = Harness(tmp_path)

    def spawn(argv, **kwargs):
        harness.bring_up("campus")
        return _ok_process()

    harness.driver._spawn = spawn
    with pytest.raises(SimulatorRestartError, match="rpc"):
        harness.driver.launch(_entry())


def test_launch_reports_the_exit_code_and_log_when_the_launcher_dies(tmp_path):
    harness = Harness(tmp_path)

    def spawn(argv, **kwargs):
        harness.exit_code = 18
        return _ok_process(exit_code=18)

    harness.driver._spawn = spawn
    with pytest.raises(SimulatorRestartError, match="exited with 18"):
        harness.driver.launch(_entry())


def test_missing_binary_is_refused_before_anything_starts(tmp_path):
    harness = Harness(tmp_path)
    entry = MapEntry(id="ghost", label="Ghost", simulator="airsim",
                     binary=str(tmp_path / "nope"), map_name="Ghost")
    with pytest.raises(SimulatorRestartError, match="missing or not executable"):
        harness.driver.start(entry)
    assert harness.spawned == []


def _ok_process(exit_code=None):
    class Fake:
        pid = 4242
        returncode = exit_code

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            return self.returncode

    return Fake()


# -- display reclaim -------------------------------------------------------
def test_stale_display_lock_is_reclaimed(tmp_path, monkeypatch):
    """The exact failure that made a map switch hang: a lock from a killed Xvfb."""
    lock, sock = display_files(":199", lock_dir=tmp_path, socket_dir=tmp_path)
    lock.write_text("999999999\n", encoding="utf-8")   # no such PID
    sock.write_text("", encoding="utf-8")
    removed = _reclaim(tmp_path, monkeypatch)
    assert removed
    assert not lock.exists() and not sock.exists()


def _reclaim(tmp_path, monkeypatch):
    import planner.simulator.processes as module

    monkeypatch.setattr(module, "display_files",
                        lambda display, lock_dir=Path("/tmp"), socket_dir=Path("/tmp/.X11-unix"):
                        (tmp_path / f".X{display[1:]}-lock", tmp_path / f"X{display[1:]}"))
    return module.reclaim_stale_display(":199")


def test_live_display_is_not_touched(tmp_path, monkeypatch):
    lock = tmp_path / ".X199-lock"
    lock.write_text(str(os.getpid()), encoding="utf-8")   # very much alive
    import planner.simulator.processes as module

    monkeypatch.setattr(module, "display_files",
                        lambda display, **kw: (lock, tmp_path / "X199"))
    with pytest.raises(SimulatorRestartError, match="still held by PID"):
        module.reclaim_stale_display(":199")
    assert lock.exists()


def test_stop_waits_for_the_xvfb_to_drop_the_display(tmp_path, monkeypatch):
    """A restart that launches immediately used to hit a half-finished teardown.

    ``xpra stop`` frees its TCP ports while the Xvfb it started still holds the X
    lock, so the next launch saw a live holder and refused with "display is still
    held by PID". Real machines show this as an intermittent map switch; the ports
    going quiet is not the end of the teardown. The wait is bounded and silent: a
    display somebody else owns on purpose still fails in launch's reclaim.
    """
    import planner.simulator.xpra_start as driver_module
    from planner.simulator import processes

    lock = tmp_path / ".X199-lock"
    sock = tmp_path / "X199"
    monkeypatch.setattr(processes, "display_files", lambda display, **kw: (lock, sock))
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    driver = harness.driver
    driver._process = SimpleNamespace(pid=4242, wait=lambda timeout: 0, poll=lambda: 0)
    lock.write_text(str(os.getpid()), encoding="utf-8")   # an Xvfb still winding down

    checks: list[bool] = []
    real_released = driver_module.display_released

    def spy(display, **kwargs):
        # The driver calls the name in its own module, so that is the one to wrap.
        result = real_released(display, **kwargs)
        checks.append(result)
        if len(checks) == 3:
            lock.unlink(missing_ok=True)                  # the Xvfb finally exits
        return result

    monkeypatch.setattr(driver_module, "display_released", spy)
    driver.stop()

    assert checks[0] is False, checks
    assert checks[-1] is True, checks
    assert len(checks) >= 2, "stop() returned before the display was free"


def test_a_stopped_adopted_stack_still_waits_for_the_display(tmp_path, monkeypatch):
    """The case the machine caught: a stack this process never spawned.

    Another HTTP process started it, so there is no child of ours to reap. Stopping
    it is still our job, and so is the teardown wait -- without it the refusal
    arrives one second later, from the next launch, looking like a stuck display.
    """
    import planner.simulator.xpra_start as driver_module
    from planner.simulator import processes

    lock = tmp_path / ".X199-lock"
    monkeypatch.setattr(processes, "display_files",
                        lambda display, **kw: (lock, tmp_path / "X199"))
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    driver = harness.driver
    assert driver._process is None, "this test is about a stack we did not spawn"
    lock.write_text(str(os.getpid()), encoding="utf-8")

    checked: list[bool] = []
    real_released = driver_module.display_released

    def spy(display, **kwargs):
        result = real_released(display, **kwargs)
        checked.append(result)
        if len(checked) == 2:
            lock.unlink(missing_ok=True)
        return result

    monkeypatch.setattr(driver_module, "display_released", spy)
    driver.stop()

    assert len(checked) >= 2, checked
    assert checked[0] is False and checked[-1] is True, checked


def _with_orphan_xvfb(harness, holder_pid, *, uid_offset=0):
    """Register the lock holder as an identity, not as a port listener.

    A real Xvfb holds no socket, so it must not appear in ``holders()`` -- putting it
    under a family port would make the driver refuse for the right reason and hide the
    behaviour under test. The identity table is keyed by port only for convenience.
    """
    harness.bindings.setdefault(59999, {})[holder_pid] = (
        "?", "/usr/bin/Xvfb",
        ["Xvfb-for-Xpra-199", "-screen", "0", "1280x720x24"],
    )
    return harness


def test_stop_reclaims_the_xvfb_xpra_leaves_behind(tmp_path, monkeypatch):
    """Measured on the machine: ``xpra stop`` killed the engine and the session and
    left ``Xvfb-for-Xpra-104`` alive with the lock, so the next launch refused with
    "display is still held by PID" -- about debris of our own stack.
    """
    import planner.simulator.xpra_start as module
    from planner.simulator import processes

    lock = tmp_path / ".X199-lock"
    monkeypatch.setattr(processes, "display_files",
                        lambda display, **kw: (lock, tmp_path / "X199"))
    holder = os.getpid()
    lock.write_text(str(holder) + "\n", encoding="utf-8")
    killed: list[int] = []
    monkeypatch.setattr(
        module, "terminate_verified_tree",
        lambda pid, before, *a, **k: killed.append(pid),
    )
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    _with_orphan_xvfb(harness, holder)

    harness.driver.stop()
    assert killed == [holder], killed


def test_a_foreign_x_server_on_the_display_is_not_killed(tmp_path, monkeypatch):
    """Same shape, different owner: refusing is correct, killing would not be.

    Only the lock holder is made foreign here. Forcing the whole harness to another
    uid would also make the engine unattributable, and then the stop would refuse at
    the ownership gate for a reason that has nothing to do with this test.
    """
    import planner.simulator.xpra_start as module
    from planner.simulator import processes

    lock = tmp_path / ".X199-lock"
    monkeypatch.setattr(processes, "display_files",
                        lambda display, **kw: (lock, tmp_path / "X199"))
    holder = os.getpid()
    lock.write_text(str(holder) + "\n", encoding="utf-8")
    killed: list[int] = []
    monkeypatch.setattr(
        module, "terminate_verified_tree",
        lambda pid, before, *a, **k: killed.append(pid),
    )
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    _with_orphan_xvfb(harness, holder)
    driver = harness.driver
    real_identity = driver._identity

    def identity(pid):
        found = real_identity(pid)
        if pid == holder:
            return (pid, os.getuid() + 1, found[2] if found else "/usr/bin/Xvfb",
                    ["Xvfb-for-Xpra-199", "-screen", "0"])
        return found

    driver._identity = identity
    driver.stop()
    assert killed == [], killed


# -- cross-family ownership ------------------------------------------------
def _both_maps():
    """One catalog, both families, both with real binary paths -- what the router
    actually hands every driver."""
    return MapCatalog(
        default_map="blocks",
        maps=(
            MapEntry(id="blocks", label="Blocks", simulator="airsim",
                     binary=BLOCKS, map_name="Blocks"),
            MapEntry(id="cityenviron", label="City", simulator="projectairsim",
                     binary=CITY, map_name="CityEnviron",
                     scene=MapScene(file="scene_city_dual.jsonc", config_path="config/scenes/city")),
        ),
    )


def test_the_other_family_stack_is_ours_to_stop(tmp_path):
    """Switching UE5 -> UE4 used to fail here.

    The AirSim driver demanded its own ``-settings=`` marker before it would call a
    listener "ours", so a running UE5 engine was an unmanaged PID, ``stop()`` refused
    to touch it, and the retry died on "display :104 is still held by PID".
    """
    engine = {91: ("cityenviron", os.path.realpath(CITY),
                   [CITY, "CityEnviron", "-windowed"])}
    harness = Harness(tmp_path, holders={RPC_PORT: engine}, alive={RPC_PORT},
                      maps=_both_maps())
    assert harness.driver.owned_map(91) is None          # not *our* map
    assert harness.driver.catalogued_map(91) == "cityenviron"   # but ours to stop
    assert [name for name, _ in harness.driver.holders().values()] == ["cityenviron"]


def test_the_markerless_family_still_names_the_airsim_map(tmp_path):
    """The other direction, for symmetry: UE5's driver stops a UE4 stack by path."""
    engine = ours_engine(91)
    harness = Harness(tmp_path, family=_ue5_family(),
                      holders={TOPICS_PORT: engine}, alive={TOPICS_PORT},
                      maps=_both_maps())
    assert [name for name, _ in harness.driver.holders().values()] == ["blocks"]


def test_start_reports_the_other_family_as_the_live_map(tmp_path):
    """The message a console operator reads when two families fight over a port."""
    engine = {91: ("cityenviron", os.path.realpath(CITY),
                   [CITY, "CityEnviron", "-windowed"])}
    maps = _both_maps()
    harness = Harness(tmp_path,
                      holders={RPC_PORT: engine, XPRA_PORT: engine},
                      alive={XPRA_PORT, RPC_PORT}, maps=maps)
    with pytest.raises(SimulatorRestartError) as raised:
        harness.driver.start(maps.entry("blocks"))
    assert "cityenviron" in str(raised.value)


def test_no_wait_when_the_display_is_already_free(tmp_path, monkeypatch):
    """Nothing holds the display here, so the reaper must not spin or sleep."""
    from planner.simulator import processes

    monkeypatch.setattr(processes, "display_files",
                        lambda display, **kw: (tmp_path / ".X199-lock", tmp_path / "X199"))
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    driver = harness.driver
    driver._process = SimpleNamespace(pid=4242, wait=lambda timeout: 0, poll=lambda: 0)
    slept: list[float] = []
    driver._sleep = slept.append
    driver.stop()
    assert slept == [], "a free display must not cost a poll"


# -- stop ladder -----------------------------------------------------------
def test_stop_uses_xpra_stop_when_we_own_the_stack(tmp_path):
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    harness.driver.stop()
    assert harness.calls[0][:2] == ["/usr/bin/xpra", "stop"]
    assert not harness.driver.running()


def test_stop_escalates_and_reports_when_ports_stay_held(tmp_path):
    harness = Harness(tmp_path, holders={RPC_PORT: ours_engine()}, alive={RPC_PORT})
    harness.stop_frees_ports = False
    # The escalation path runs a verified tree kill on a PID that does not exist,
    # which lands in ProcessLookupError and is swallowed by design.
    with pytest.raises(SimulatorRestartError, match="still holds ports"):
        harness.driver.stop()
    assert harness.calls[0][:2] == ["/usr/bin/xpra", "stop"]


def test_stop_is_a_noop_when_nothing_is_listening(tmp_path):
    harness = Harness(tmp_path)
    harness.driver.stop()
    assert harness.calls == []


def test_restart_stops_then_launches(tmp_path):
    harness = Harness(tmp_path)
    harness.bring_up()

    def spawn(argv, **kwargs):
        harness.spawned.append((argv, kwargs))
        harness.bring_up("campus")
        return _ok_process()

    harness.driver._spawn = spawn
    harness.driver.restart(_entry("campus"))
    assert len(harness.spawned) == 1
    assert harness.calls[0][:2] == ["/usr/bin/xpra", "stop"]
    assert harness.driver.running()


def test_start_on_the_live_map_is_not_an_error(tmp_path):
    harness = Harness(tmp_path)
    harness.bring_up()
    harness.driver.start(_entry("blocks"))
    assert harness.spawned == []


def test_start_over_another_live_map_is_refused(tmp_path):
    harness = Harness(tmp_path)
    harness.bring_up("campus")
    with pytest.raises(SimulatorRestartError, match="another map is live"):
        harness.driver.start(_entry("blocks"))


def test_launch_writes_the_map_marker(tmp_path):
    harness = Harness(tmp_path)

    def spawn(argv, **kwargs):
        harness.bring_up("campus")
        return _ok_process()

    harness.driver._spawn = spawn
    harness.driver.launch(_entry("campus"))
    marker = Path(str(harness.driver._marker_file))
    assert marker.read_text(encoding="utf-8").strip() == "campus"


# -- probes and wiring -----------------------------------------------------
def test_unknown_probe_is_refused_at_construction(tmp_path):
    with pytest.raises(ValueError, match="probe"):
        _ = Harness(tmp_path, family=_family(probes=("ports", "telepathy"))).driver


def test_rpc_probe_needs_a_port_named_rpc(tmp_path):
    with pytest.raises(ValueError, match="no port named rpc"):
        _ = Harness(tmp_path, family=_family(probes=("rpc",), ports={"xpra": XPRA_PORT})).driver


def test_window_probe_is_supported():
    assert "window" in SUPPORTED_PROBES
