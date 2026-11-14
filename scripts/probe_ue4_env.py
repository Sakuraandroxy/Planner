#!/usr/bin/env python3
"""Check the machine before the first UE4 bring-up.

Run this first, from the repository root:

    python scripts/probe_ue4_env.py

It needs only the standard library and PyYAML, so it still runs in a half-built
environment and tells you what is missing there. Every check prints one line:
``ok``, ``warn`` (works but read it) or ``FAIL``. Exit code is 1 if any check
failed.

Checks cover what the plan says cannot be verified off the target machine: the
six AirSim binaries, the AirSim settings file, the port the planner will dial,
whether the camera name in settings agrees with the planner config, and whether
the two simulator families' Python dependencies can coexist.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import socket
import sys

ROOT = Path(__file__).resolve().parents[1]

#: import name -> pip name. They differ, and the hint must be installable.
CORE_PACKAGES = {
    "fastapi": "fastapi",
    "uvicorn": "uvicorn[standard]",
    "pydantic": "pydantic",
    "numpy": "numpy",
    "cv2": "opencv-python-headless",
    "httpx": "httpx",
    "yaml": "pyyaml",
}
UE4_PACKAGES = {
    "airsim": "airsim>=1.8.1",
    "msgpackrpc": "msgpack-rpc-python>=0.4.1",
    "PIL": "Pillow>=10.0.0",
    "requests": "requests>=2.31.0",
}
UE5_PACKAGES = {"projectairsim": "the ProjectAirSim wheel", "pynng": "pynng"}

results: list[tuple[str, str, str]] = []


def report(kind: str, name: str, detail: str = "") -> None:
    results.append((kind, name, detail))
    print(f"[{kind:<4}] {name}" + (f": {detail}" if detail else ""))


def ok(name, detail=""):
    report("ok", name, detail)


def warn(name, detail=""):
    report("warn", name, detail)


def fail(name, detail=""):
    report("FAIL", name, detail)


def have(module: str) -> bool:
    try:
        return importlib.util.find_spec(module) is not None
    except (ImportError, ValueError):
        return False


def check_python() -> None:
    version = sys.version_info
    if version < (3, 10):
        fail("python", f"{sys.version.split()[0]} is older than the 3.10 floor")
    else:
        ok("python", sys.version.split()[0])


def check_packages() -> None:
    for label, packages, required in (
        ("core", CORE_PACKAGES, True),
        ("ue4", UE4_PACKAGES, True),
        ("ue5", UE5_PACKAGES, False),
    ):
        missing = [name for name in packages if not have(name)]
        if missing:
            pip_names = " ".join(packages[name].split("[")[0] for name in missing)
            (fail if required else warn)(
                f"{label} packages",
                f"missing {missing} — pip install {pip_names}",
            )
        else:
            ok(f"{label} packages", ", ".join(packages))
    if have("airsim") and have("projectairsim"):
        warn(
            "client coexistence",
            "both airsim and projectairsim are importable in ONE interpreter; "
            "the UE4 runtime must run in its own environment so the old "
            "msgpack-rpc Tornado cannot collide",
        )


def load_yaml(path: Path):
    import yaml

    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def check_repo_layout() -> None:
    for rel in ("config/base.yaml", "config/maps.yaml", "config/simulators.yaml"):
        path = ROOT / rel
        ok(rel) if path.is_file() else fail(rel, "file is missing")
    ue4 = ROOT / "planner/ue4"
    if ue4.is_dir() and (ue4 / "server_worker.py").is_file():
        ok("planner/ue4", "copied UE4 stack present")
    else:
        fail("planner/ue4", "the copied UE4 stack is missing from this checkout")
    console = ROOT / "console/index.html"
    ok("console/index.html") if console.is_file() else fail(
        "console/index.html", "the console is missing; / will 404"
    )


def check_families_and_maps() -> tuple[dict, list[dict]]:
    families_raw = load_yaml(ROOT / "config/simulators.yaml")
    maps_raw = load_yaml(ROOT / "config/maps.yaml")
    families = {
        key: value
        for key, value in families_raw.items()
        if isinstance(value, dict)
    }
    maps = [entry for entry in (maps_raw.get("maps") or []) if isinstance(entry, dict)]
    known = set(families)
    orphans = sorted({str(m.get("family")) for m in maps} - known - {"None"})
    if orphans:
        fail("map families", f"maps reference unconfigured families {orphans}")
    else:
        ok("map families", ", ".join(sorted(known)))
    return families, maps


def check_ue4_binaries(maps: list[dict]) -> None:
    ue4_maps = [entry for entry in maps if entry.get("family") == "ue4"]
    if not ue4_maps:
        warn("ue4 maps", "config/maps.yaml lists no family ue4 maps")
        return
    for entry in ue4_maps:
        binary = str(entry.get("binary", ""))
        name = str(entry.get("id", binary))
        if not binary:
            fail(f"map {name}", "no binary path")
            continue
        path = Path(binary)
        if not path.is_file():
            fail(f"map {name}", f"binary missing: {binary}")
            continue
        if not os.access(path, os.X_OK):
            fail(f"map {name}", f"binary not executable: {binary}")
            continue
        ok(f"map {name}", path.name)


def check_settings(families: dict) -> None:
    ue4 = families.get("ue4", {}) or {}
    raw = str(ue4.get("settings_file", ""))
    settings = Path(raw)
    if not raw:
        fail("ue4 settings_file", "not configured")
        return
    if not settings.is_file():
        fail("ue4 settings_file", f"missing: {raw}")
        return
    import json

    try:
        data = json.loads(settings.read_text(encoding="utf-8"))
    except ValueError as error:
        fail("ue4 settings_file", f"not valid JSON: {error}")
        return
    port = data.get("ApiServerPort")
    declared = (ue4.get("ports") or {}).get("rpc")
    if port is None:
        warn("ApiServerPort", "absent, AirSim will default to 41451")
    elif declared is not None and int(port) != int(declared):
        fail(
            "ApiServerPort",
            f"settings says {port} but simulators.yaml ue4 says {declared}; "
            "the planner would dial the wrong port",
        )
    else:
        ok("ApiServerPort", str(port))
    vehicles = data.get("Vehicles") or {}
    if not vehicles:
        warn("settings Vehicles", "empty; AirSim exposes no drone to control")
    else:
        names = list(vehicles)
        ok("settings Vehicles", ", ".join(str(name) for name in names))
        camera_hits = sorted(
            {
                str(camera)
                for vehicle in vehicles.values()
                if isinstance(vehicle, dict)
                for camera in (vehicle.get("Cameras") or {})
            }
        )
        if camera_hits:
            ok("settings cameras", ", ".join(camera_hits))
        if "front_center" not in camera_hits:
            warn(
                "settings cameras",
                "front_center is absent; the UE4 observation source names it",
            )


def check_ports(families: dict) -> None:
    wanted: dict[str, int] = {}
    for family_id, item in families.items():
        for label, port in ((item.get("ports") or {})).items():
            wanted[f"{family_id}.{label}"] = int(port)
    if not wanted:
        warn("ports", "no ports declared in simulators.yaml")
        return
    for name, port in sorted(wanted.items()):
        state = "in use" if socket_listen(port) else "free"
        (warn if state == "in use" else ok)(f"port {name}", f"{port} {state}")
    print(
        "        note: 'in use' is expected while a stack runs; the planner "
        "refuses to stop a stack it did not start"
    )


def socket_listen(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.25)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def check_xpra() -> None:
    from shutil import which

    found = which("xpra")
    if found is None:
        fail("xpra", "not installed; UE4 scenes launch through xpra start")
        return
    if found != "/usr/bin/xpra":
        warn("xpra", f"found {found}, but simulators.yaml argv starts with /usr/bin/xpra")
        return
    ok("xpra", found)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    print(f"repository: {ROOT}")
    print(f"python    : {sys.executable}")
    check_python()
    check_packages()
    check_repo_layout()
    families, maps = check_families_and_maps()
    check_ue4_binaries(maps)
    check_settings(families)
    check_xpra()
    check_ports(families)
    counts = {kind: sum(1 for item in results if item[0] == kind) for kind in ("ok", "warn", "FAIL")}
    print(
        f"\nsummary: {counts['ok']} ok, {counts['warn']} warn, {counts['FAIL']} failed"
    )
    if counts["FAIL"]:
        print("fix every FAIL before starting the planner; see docs/UE4_BRINGUP.md")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
