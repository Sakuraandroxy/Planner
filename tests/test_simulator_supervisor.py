from pathlib import Path
from types import SimpleNamespace

import pytest

from planner.adapters.runtime.simulator_supervisor import (
    SimulatorSupervisor,
    read_map_marker,
)
from planner.config import MapCatalog, MapEntry, SimulatorSettings
from planner.errors import SimulatorNotOwned, SimulatorRestartError

DISPLAY = ":199"
XPRA_PORT = 20010
TOPICS_PORT = 20991
SERVICES_PORT = 20992


def _catalog():
    return MapCatalog(
        default_map="city",
        maps=(
            MapEntry("city", "城市", "cityenviron", "scene_city.jsonc",
                     "config/scenes/city"),
            MapEntry("suburb", "郊区", "neighborhood",
                     "scene_neighborhood.jsonc",
                     "config/scenes/neighborhood"),
        ),
    )


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def _settings(tmp_path, **overrides):
    values = dict(
        display=DISPLAY,
        xpra_port=XPRA_PORT,
        topics_port=TOPICS_PORT,
        services_port=SERVICES_PORT,
        runtime_dir=str(tmp_path / "output/xpra"),
        map_marker_file=str(tmp_path / "output/xpra/current_map"),
        launch_timeout_s=180.0,
        stop_timeout_s=40.0,
        port_release_timeout_s=5.0,
    )
    values.update(overrides)
    return SimulatorSettings(**values)


def _make_project(tmp_path):
    (tmp_path / "scripts").mkdir(parents=True, exist_ok=True)
    (tmp_path / "scripts/launch_projectairsim_xpra.sh").write_text(
        "#!/bin/sh\n", encoding="utf-8"
    )
    (tmp_path / "scripts/stop_projectairsim_xpra.sh").write_text(
        "#!/bin/sh\n", encoding="utf-8"
    )
    (tmp_path / "output/xpra").mkdir(parents=True, exist_ok=True)
    (tmp_path / "sim").mkdir(parents=True, exist_ok=True)


def _write_pid(tmp_path, name, pid):
    (tmp_path / "output/xpra" / name).write_text(f"{pid}\n", encoding="utf-8")


class FakeRunner:
    def __init__(self, on_call=None, returncode=0, stderr=""):
        self.calls = []
        self.on_call = on_call
        self.returncode = returncode
        self.stderr = stderr

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        if self.on_call is not None:
            self.on_call(argv)
        return SimpleNamespace(
            returncode=self.returncode, stdout="", stderr=self.stderr
        )


def _supervisor(tmp_path, *, ports=None, cmdlines=None, runner=None):
    clock = Clock()
    listening = set() if ports is None else set(ports)
    cmdlines = {} if cmdlines is None else dict(cmdlines)
    runner = FakeRunner() if runner is None else runner

    def port_checker(port):
        return port in listening

    def cmdline_reader(pid):
        return cmdlines.get(pid)

    supervisor = SimulatorSupervisor(
        _settings(tmp_path),
        tmp_path,
        runner=runner,
        port_checker=port_checker,
        clock=clock,
        sleep=clock.sleep,
        cmdline_reader=cmdline_reader,
    )
    supervisor.listening = listening
    return supervisor


def test_running_requires_every_stack_port(tmp_path):
    supervisor = _supervisor(tmp_path)
    assert not supervisor.running()
    supervisor.listening.update({XPRA_PORT, TOPICS_PORT})
    assert not supervisor.running()
    supervisor.listening.add(SERVICES_PORT)
    assert supervisor.running()


def test_launch_command_lists_the_script_arguments(tmp_path):
    supervisor = _supervisor(tmp_path)
    entry = _catalog().entry("suburb")
    assert supervisor.launch_command(entry) == (
        f"scripts/launch_projectairsim_xpra.sh neighborhood {DISPLAY} "
        f"{XPRA_PORT} {TOPICS_PORT} {SERVICES_PORT}"
    )


def test_preflight_rejects_a_stack_without_pidfiles(tmp_path):
    _make_project(tmp_path)
    supervisor = _supervisor(tmp_path)
    with pytest.raises(SimulatorNotOwned) as error:
        supervisor.preflight()
    message = str(error.value)
    assert "refusing to stop" in message
    assert "scripts/launch_projectairsim_xpra.sh" in message
    assert DISPLAY in message


def test_preflight_accepts_processes_that_belong_to_the_project(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    _write_pid(tmp_path, f"shadow{DISPLAY[1:]}.pid", 3333)
    supervisor = _supervisor(
        tmp_path,
        cmdlines={
            1111: f"{tmp_path}/sim/Binaries/UE {DISPLAY}",
            2222: f"Xvfb {DISPLAY} -screen 0 1920x1080x24",
            3333: f"xpra shadow {DISPLAY}",
        },
    )
    supervisor.preflight()


def test_preflight_rejects_a_foreign_pid(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 4444)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 5555)
    supervisor = _supervisor(
        tmp_path,
        cmdlines={
            4444: "python -m some_other_tool",
            5555: f"Xvfb {DISPLAY}",
        },
    )
    with pytest.raises(SimulatorNotOwned, match="simulator PID 4444"):
        supervisor.preflight()


def test_preflight_rejects_missing_scripts(tmp_path):
    (tmp_path / "output/xpra").mkdir(parents=True, exist_ok=True)
    supervisor = _supervisor(tmp_path)
    with pytest.raises(SimulatorRestartError, match="script not found"):
        supervisor.preflight()


def test_stop_runs_the_stop_script_and_waits_for_release(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    supervisor = _supervisor(
        tmp_path,
        ports={XPRA_PORT, TOPICS_PORT, SERVICES_PORT},
        cmdlines={1111: "sim", 2222: f"Xvfb {DISPLAY}"},
    )

    def release(argv):
        supervisor.listening.clear()
        for name in (f"sim{DISPLAY[1:]}.pid", f"xvfb{DISPLAY[1:]}.pid"):
            (tmp_path / "output/xpra" / name).unlink()

    runner = FakeRunner(on_call=release)
    supervisor._runner = runner
    supervisor.stop()

    assert runner.calls[0][0] == [
        str(tmp_path / "scripts/stop_projectairsim_xpra.sh"),
        DISPLAY,
    ]
    assert runner.calls[0][1]["cwd"] == str(tmp_path)
    assert runner.calls[0][1]["env"]["PROJECTAIRSIM_XPRA_RUNTIME"] == str(
        tmp_path / "output/xpra"
    )


def test_stop_times_out_when_the_ports_stay_open(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    supervisor = _supervisor(
        tmp_path,
        ports={XPRA_PORT, TOPICS_PORT, SERVICES_PORT},
        cmdlines={1111: "sim", 2222: f"Xvfb {DISPLAY}"},
    )
    with pytest.raises(SimulatorRestartError, match="did not stop"):
        supervisor.stop()


def test_stop_reports_the_script_failure_output(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    supervisor = _supervisor(
        tmp_path,
        cmdlines={1111: "sim", 2222: f"Xvfb {DISPLAY}"},
    )
    supervisor._runner = FakeRunner(returncode=1, stderr="cannot stop")
    with pytest.raises(SimulatorRestartError) as error:
        supervisor.stop()
    assert "exited with 1" in str(error.value)
    assert "cannot stop" in str(error.value)


def test_launch_passes_the_map_and_ports_then_writes_the_marker(tmp_path):
    _make_project(tmp_path)
    supervisor = _supervisor(tmp_path)

    def open_ports(argv):
        supervisor.listening.update(
            {XPRA_PORT, TOPICS_PORT, SERVICES_PORT}
        )

    runner = FakeRunner(on_call=open_ports)
    supervisor._runner = runner
    entry = _catalog().entry("suburb")
    supervisor.launch(entry)

    assert runner.calls[0][0] == [
        str(tmp_path / "scripts/launch_projectairsim_xpra.sh"),
        "neighborhood",
        DISPLAY,
        str(XPRA_PORT),
        str(TOPICS_PORT),
        str(SERVICES_PORT),
    ]
    marker = tmp_path / "output/xpra/current_map"
    assert marker.read_text(encoding="utf-8").strip() == "suburb"


def test_start_launches_when_no_stack_is_running(tmp_path):
    _make_project(tmp_path)
    supervisor = _supervisor(tmp_path)

    def open_ports(argv):
        supervisor.listening.update(
            {XPRA_PORT, TOPICS_PORT, SERVICES_PORT}
        )

    supervisor._runner = FakeRunner(on_call=open_ports)
    supervisor.start(_catalog().entry("suburb"))
    marker = tmp_path / "output/xpra/current_map"
    assert marker.read_text(encoding="utf-8").strip() == "suburb"


def test_start_refuses_when_a_stack_is_already_running(tmp_path):
    _make_project(tmp_path)
    supervisor = _supervisor(
        tmp_path, ports={XPRA_PORT, TOPICS_PORT, SERVICES_PORT}
    )
    with pytest.raises(SimulatorRestartError, match="already running"):
        supervisor.start(_catalog().entry("suburb"))


def test_launch_times_out_without_ports_and_writes_no_marker(tmp_path):
    _make_project(tmp_path)
    supervisor = _supervisor(tmp_path)
    entry = _catalog().entry("suburb")
    with pytest.raises(SimulatorRestartError, match="did not open ports"):
        supervisor.launch(entry)
    assert not (tmp_path / "output/xpra/current_map").exists()


def test_restart_stops_before_launching_and_writes_the_marker_last(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    supervisor = _supervisor(
        tmp_path,
        ports={XPRA_PORT, TOPICS_PORT, SERVICES_PORT},
        cmdlines={
            1111: f"{tmp_path}/sim/Binaries/UE",
            2222: f"Xvfb {DISPLAY}",
        },
    )
    marker = tmp_path / "output/xpra/current_map"
    observed = {}

    def runner(argv, **kwargs):
        if str(argv[0]).endswith("stop_projectairsim_xpra.sh"):
            observed["marker_at_stop"] = marker.exists()
            supervisor.listening.clear()
            for name in (
                f"sim{DISPLAY[1:]}.pid",
                f"xvfb{DISPLAY[1:]}.pid",
            ):
                (tmp_path / "output/xpra" / name).unlink()
        else:
            observed["marker_at_launch"] = marker.exists()
            supervisor.listening.update(
                {XPRA_PORT, TOPICS_PORT, SERVICES_PORT}
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    supervisor._runner = runner
    supervisor.restart(_catalog().entry("suburb"))

    assert observed == {"marker_at_stop": False, "marker_at_launch": False}
    assert marker.read_text(encoding="utf-8").strip() == "suburb"


def test_restart_keeps_xpra_and_restarts_only_the_simulator(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    supervisor = _supervisor(
        tmp_path,
        ports={XPRA_PORT, TOPICS_PORT, SERVICES_PORT},
        cmdlines={
            1111: f"{tmp_path}/sim/Binaries/UE",
            2222: f"Xvfb {DISPLAY}",
        },
    )
    marker = tmp_path / "output/xpra/current_map"
    observed = {}

    def runner(argv, **kwargs):
        if str(argv[0]).endswith("stop_projectairsim_xpra.sh"):
            observed["stop_args"] = list(argv)
            observed["marker_at_stop"] = marker.exists()
            # Sim-only stop leaves xpra listening; only the UE ports close.
            supervisor.listening.discard(TOPICS_PORT)
            supervisor.listening.discard(SERVICES_PORT)
            (tmp_path / "output/xpra" / f"sim{DISPLAY[1:]}.pid").unlink()
            (tmp_path / "output/xpra" / f"xvfb{DISPLAY[1:]}.pid").unlink()
        else:
            observed["launch_args"] = list(argv)
            observed["marker_at_launch"] = marker.exists()
            supervisor.listening.update({TOPICS_PORT, SERVICES_PORT})
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    supervisor._runner = runner
    supervisor.restart(_catalog().entry("suburb"))

    assert observed["marker_at_stop"] is False
    assert observed["marker_at_launch"] is False
    assert observed["stop_args"] == [
        str(tmp_path / "scripts/stop_projectairsim_xpra.sh"),
        DISPLAY,
        "--sim-only",
    ]
    assert observed["launch_args"] == [
        str(tmp_path / "scripts/launch_projectairsim_xpra.sh"),
        "neighborhood",
        DISPLAY,
        str(XPRA_PORT),
        str(TOPICS_PORT),
        str(SERVICES_PORT),
        "--sim-only",
    ]
    assert marker.read_text(encoding="utf-8").strip() == "suburb"


def test_restart_falls_back_to_the_full_stack_when_xpra_is_down(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    supervisor = _supervisor(
        tmp_path,
        ports={TOPICS_PORT, SERVICES_PORT},
        cmdlines={
            1111: f"{tmp_path}/sim/Binaries/UE",
            2222: f"Xvfb {DISPLAY}",
        },
    )
    observed = {}

    def runner(argv, **kwargs):
        if str(argv[0]).endswith("stop_projectairsim_xpra.sh"):
            observed["stop_args"] = list(argv)
            supervisor.listening.clear()
            for name in (
                f"sim{DISPLAY[1:]}.pid",
                f"xvfb{DISPLAY[1:]}.pid",
            ):
                (tmp_path / "output/xpra" / name).unlink()
        else:
            observed["launch_args"] = list(argv)
            supervisor.listening.update(
                {XPRA_PORT, TOPICS_PORT, SERVICES_PORT}
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    supervisor._runner = runner
    supervisor.restart(_catalog().entry("suburb"))

    assert observed["stop_args"] == [
        str(tmp_path / "scripts/stop_projectairsim_xpra.sh"),
        DISPLAY,
    ]
    assert observed["launch_args"] == [
        str(tmp_path / "scripts/launch_projectairsim_xpra.sh"),
        "neighborhood",
        DISPLAY,
        str(XPRA_PORT),
        str(TOPICS_PORT),
        str(SERVICES_PORT),
    ]


def test_stop_sim_only_leaves_xpra_listening(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    supervisor = _supervisor(
        tmp_path,
        ports={XPRA_PORT, TOPICS_PORT, SERVICES_PORT},
        cmdlines={1111: "sim"},
    )

    def release(argv):
        assert argv[-1] == "--sim-only"
        supervisor.listening.discard(TOPICS_PORT)
        supervisor.listening.discard(SERVICES_PORT)
        (tmp_path / "output/xpra" / f"sim{DISPLAY[1:]}.pid").unlink()

    runner = FakeRunner(on_call=release)
    supervisor._runner = runner
    supervisor.stop(sim_only=True)

    assert XPRA_PORT in supervisor.listening


def test_restart_failure_keeps_the_previous_marker(tmp_path):
    _make_project(tmp_path)
    _write_pid(tmp_path, f"sim{DISPLAY[1:]}.pid", 1111)
    _write_pid(tmp_path, f"xvfb{DISPLAY[1:]}.pid", 2222)
    marker = tmp_path / "output/xpra/current_map"
    marker.write_text("city\n", encoding="utf-8")
    supervisor = _supervisor(
        tmp_path,
        ports={XPRA_PORT, TOPICS_PORT, SERVICES_PORT},
        cmdlines={
            1111: f"{tmp_path}/sim/Binaries/UE",
            2222: f"Xvfb {DISPLAY}",
        },
    )
    with pytest.raises(SimulatorRestartError):
        supervisor.restart(_catalog().entry("suburb"))
    assert marker.read_text(encoding="utf-8").strip() == "city"


def test_read_map_marker_only_accepts_catalog_ids(tmp_path):
    marker = tmp_path / "current_map"
    assert read_map_marker(marker, _catalog()) is None
    marker.write_text("suburb\n", encoding="utf-8")
    assert read_map_marker(marker, _catalog()) == "suburb"
    marker.write_text("moon\n", encoding="utf-8")
    assert read_map_marker(marker, _catalog()) is None
