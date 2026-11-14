#!/usr/bin/env python3
"""Record the streaming-view cycle on a live ProjectAirSim map.

ProjectAirSim has no free-spectator camera: ``World.switch_streaming_view()``
cycles through the robots' sensors marked ``streaming-enabled`` in their robot
JSONC. The cycle order is not stable across builds or scene loads, so the
session measures it at startup (``planner/adapters/projectairsim/
streaming_views.py``) by matching xpra screenshots against ``GetImages``
frames. This script is the manual evidence tool for that mapping: it walks the
catalog through ``POST /api/view``, captures after each step and reports frame
differences. Open the PNGs in ``--out`` to confirm that, for example,
``rover_front`` really shows the rover nose camera. No source edit is needed
when the order differs; the runtime calibration follows the live server.

The script requires an idle planner and a running xpra display.

Usage:
    python scripts/calibrate_streaming_views.py --display :104
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from planner.domain.view import VIEW_MODES  # noqa: E402


def _request_json(url: str, method: str = "GET", payload=None):
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = Request(url, data=data, headers=headers, method=method)
    with urlopen(request, timeout=10) as response:
        return json.loads(response.read().decode("utf-8"))


def _wait_idle(base_url: str, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            status = _request_json(f"{base_url}/api/status")
        except URLError as error:
            raise SystemExit(f"planner is not reachable at {base_url}: {error}")
        if status.get("state") == "idle":
            return status
        if time.monotonic() >= deadline:
            raise SystemExit(
                f"planner stayed {status.get('state')!r} for {timeout_s:g}s; "
                "wait until it is idle and rerun"
            )
        time.sleep(1.0)


def _screenshot(display: str, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(
        ["xpra", "screenshot", str(path), display],
        capture_output=True,
        text=True,
        timeout=20,
    )
    if completed.returncode != 0 or not path.is_file():
        raise SystemExit(
            f"xpra screenshot failed: {completed.stderr.strip() or completed.stdout.strip()}"
        )


def _frame_difference(first: Path, second: Path) -> float:
    """Mean absolute grayscale difference in [0, 1]; 0 means identical."""
    import cv2
    import numpy as np

    left = cv2.imread(str(first), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(str(second), cv2.IMREAD_GRAYSCALE)
    if left is None or right is None:
        raise SystemExit(f"could not decode screenshots {first} / {second}")
    if left.shape != right.shape:
        right = cv2.resize(right, (left.shape[1], left.shape[0]))
    delta = np.abs(left.astype(np.float32) - right.astype(np.float32))
    return float(delta.mean()) / 255.0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url", default="http://127.0.0.1:8000", help="planner base URL"
    )
    parser.add_argument(
        "--display", default=":104", help="X display served by xpra shadow"
    )
    parser.add_argument(
        "--out",
        default=str(PROJECT_DIR / "output/calibration"),
        help="directory for the captured frames and summary.json",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=1.0,
        help="wait after each view request before the screenshot",
    )
    parser.add_argument(
        "--idle-timeout-s",
        type=float,
        default=180.0,
        help="how long to wait for an idle planner",
    )
    args = parser.parse_args(argv)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    status = _wait_idle(args.url, args.idle_timeout_s)
    map_id = status.get("map") or "unknown"
    print(f"planner idle on map {map_id!r}; calibrating {len(VIEW_MODES)} views")

    summary = {"map": map_id, "display": args.display, "captures": []}
    initial_file = out_dir / "00_initial.png"
    _screenshot(args.display, initial_file)
    print(f"  initial frame -> {initial_file}")
    previous = initial_file
    previous_mode = "initial"

    for index, mode in enumerate(VIEW_MODES, start=1):
        body = _request_json(
            f"{args.url}/api/view", method="POST", payload={"mode": mode}
        )
        if body.get("deferred"):
            raise SystemExit(
                f"planner deferred the view request to {mode!r}; the console "
                "was not idle"
            )
        time.sleep(max(0.0, args.settle_s))
        path = out_dir / f"{index:02d}_{mode}.png"
        _screenshot(args.display, path)
        difference = _frame_difference(previous, path)
        summary["captures"].append(
            {
                "mode": mode,
                "file": str(path),
                "diff_from_previous": round(difference, 5),
                "previous_mode": previous_mode,
            }
        )
        print(
            f"  {mode:<12} -> {path}  diff({previous_mode}->{mode})="
            f"{difference:.4f}"
        )
        previous, previous_mode = path, mode

    if previous_mode != VIEW_MODES[0]:
        _request_json(
            f"{args.url}/api/view", method="POST", payload={"mode": VIEW_MODES[0]}
        )
        time.sleep(max(0.0, args.settle_s))
        wrap_file = out_dir / f"{len(VIEW_MODES) + 1:02d}_wrap_{VIEW_MODES[0]}.png"
        _screenshot(args.display, wrap_file)
        wrap_diff = _frame_difference(initial_file, wrap_file)
        summary["wrap_diff_from_initial"] = round(wrap_diff, 5)
        print(
            f"  wrap back to {VIEW_MODES[0]} -> {wrap_file}  "
            f"diff(initial)={wrap_diff:.4f}"
        )

    summary["status_after"] = _request_json(f"{args.url}/api/status")
    summary_file = out_dir / "summary.json"
    summary_file.write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(f"summary -> {summary_file}")

    tiny = [
        item
        for item in summary["captures"]
        if item["diff_from_previous"] < 0.001
    ]
    if tiny:
        print(
            "warning: consecutive frames look identical for "
            + ", ".join(item["mode"] for item in tiny)
            + "; check that all four cameras are streaming-enabled"
        )
        return 1
    print(
        "open the PNGs to confirm each camera matches its label; the session "
        "calibrates the same mapping automatically at startup"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
