#!/usr/bin/env python3
"""Fly one instruction through the planner API and record what the stage does.

    python3 scripts/probe_flight.py --instruction "飞到左前方那辆白色汽车旁边" \
        --timeout 120 > run.jsonl
Standard library only, so it runs in either interpreter on the target machine and
needs no planner import. The flow is the console's: read ``/api/status``, start
the scene through ``/api/world`` if none is up, wait for it to become ``idle``,
switch the profile to ``vlm``, submit the task through ``/api/tasks``, poll
``/api/status`` and ``/api/trajectory`` once a second, then stop the scene with
``/api/world/stop``.

Output: one JSON object per poll on stdout, one per second, flushed as it is
measured, so a redirected file survives a kill::

    {"t": 0.0, "state": "running", "stage": "stage_1", "status": null,
     "pose": {"x": 0.0, "y": -16.0, "z": -3.59, "yaw_deg": 0.0}, "distance": 3.2}

``t`` is seconds since the task was accepted. ``stage``, ``pose`` and ``distance``
come from the trajectory snapshot, which the workflow publishes once per planning
round: between rounds these fields repeat the last published values instead of
inventing a fresher pose, because the API exposes no other. ``distance`` is the
straight-line horizontal distance from that published pose to the last waypoint
of the published batch, and null when no waypoint is queued. ``status`` is the
task's own result text, null while it runs.

The summary goes to stderr so stdout stays valid JSONL.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request

#: Per-request limit. A poll that takes longer than this has stopped being a
#: once-a-second sample.
REQUEST_TIMEOUT_S = 10.0

#: How long the scene may take to come up (or to come back after a profile
#: switch). The simulator owns this: the launch script allows 180 s and the
#: worker 60 s to connect.
READY_TIMEOUT_S = 300.0

POLL_INTERVAL_S = 1.0


def _request(base: str, method: str, path: str, body: dict | None = None):
    data = None if body is None else json.dumps(body).encode()
    headers = {} if data is None else {"Content-Type": "application/json"}
    request = urllib.request.Request(
        base + path, data=data, method=method, headers=headers
    )
    try:
        with urllib.request.urlopen(request, timeout=REQUEST_TIMEOUT_S) as answer:
            payload = answer.read().decode()
    except urllib.error.HTTPError as error:
        detail = error.read().decode(errors="replace")
        raise SystemExit(f"{method} {path} answered {error.code}: {detail}")
    except urllib.error.URLError as error:
        raise SystemExit(f"{method} {path} failed: {error.reason}")
    return json.loads(payload or "null")


def _get(base: str, path: str):
    return _request(base, "GET", path)


def _post(base: str, path: str, body: dict | None = None):
    return _request(base, "POST", path, body)


def _wait_ready(base: str) -> dict:
    """Poll until the scene is connected and idle; the state machine owns the wait."""
    deadline = time.monotonic() + READY_TIMEOUT_S
    while True:
        status = _get(base, "/api/status")
        if status.get("state") == "idle":
            return status
        if status.get("sim_error"):
            raise SystemExit(f"the simulator failed: {status['sim_error']}")
        if time.monotonic() >= deadline:
            raise SystemExit(
                f"the scene did not become idle in {READY_TIMEOUT_S:g}s "
                f"(state={status.get('state')})"
            )
        time.sleep(POLL_INTERVAL_S)


def _prepare(base: str) -> dict:
    """Bring the scene up and select the profile the probe flies with."""
    status = _get(base, "/api/status")
    if status.get("state") != "idle":
        options = _get(base, "/api/world/options")
        selected = options.get("selection") or {}
        map_id = selected.get("map") or options.get("default_map")
        if not map_id:
            raise SystemExit("no map is selected and the catalog names no default")
        print(f"starting the scene {map_id}", file=sys.stderr)
        _post(base, "/api/world", {"map": map_id})
        status = _wait_ready(base)
    if status.get("profile") != "vlm":
        print("switching the profile to vlm", file=sys.stderr)
        _post(base, "/api/profile", {"profile": "vlm"})
        status = _wait_ready(base)
    return status


def _sample(elapsed_s: float, status: dict, snapshot: dict) -> dict:
    vehicle = snapshot.get("vehicle")
    points = snapshot.get("points") or ()
    distance = None
    if vehicle is not None and points:
        goal = points[-1]
        distance = round(
            math.hypot(goal["x"] - vehicle["x"], goal["y"] - vehicle["y"]), 3
        )
    return {
        "t": round(elapsed_s, 3),
        "state": status.get("state"),
        "stage": snapshot.get("stage_id"),
        "status": status.get("last_result"),
        "pose": None
        if vehicle is None
        else {
            "x": round(vehicle["x"], 3),
            "y": round(vehicle["y"], 3),
            "z": round(vehicle["z"], 3),
            "yaw_deg": round(vehicle["yaw_deg"], 2),
        },
        "distance": distance,
    }


def _fly(base: str, instruction: str, timeout_s: float) -> tuple[list[dict], dict]:
    """Submit the task and record it once a second until it ends or time is up."""
    accepted = _post(base, "/api/tasks", {"instruction": instruction})
    print(f"accepted {accepted.get('task_id')} for {accepted.get('vehicle')}",
          file=sys.stderr)
    started = time.monotonic()
    records: list[dict] = []
    snapshot: dict = {}
    while True:
        status = _get(base, "/api/status")
        snapshot = _get(base, "/api/trajectory")
        record = _sample(time.monotonic() - started, status, snapshot)
        records.append(record)
        print(json.dumps(record, ensure_ascii=False), flush=True)
        # A submitted task runs and then ends in ``stopping`` or ``idle``; a run
        # cut short by the deadline keeps the last sample it took.
        if status.get("state") not in ("running", "stopping"):
            break
        if time.monotonic() - started >= timeout_s:
            break
        time.sleep(POLL_INTERVAL_S)
    elapsed = time.monotonic() - started
    _summary(
        instruction, accepted.get("task_id"), records, snapshot, elapsed, timeout_s
    )
    return records, snapshot


def _summary(instruction: str, task_id, records: list[dict], snapshot: dict,
             elapsed_s: float, timeout_s: float) -> None:
    last = records[-1] if records else {}
    print("--- summary ---", file=sys.stderr)
    print(f"instruction: {instruction}", file=sys.stderr)
    print(f"task: {task_id}", file=sys.stderr)
    print(
        f"ended: state={last.get('state')} status={last.get('status')!r} "
        f"after {elapsed_s:.1f}s (limit {timeout_s:g}s)",
        file=sys.stderr,
    )
    print(f"samples: {len(records)}", file=sys.stderr)
    print(
        f"planning_calls: {snapshot.get('planning_calls')} "
        f"path_length_m: {snapshot.get('path_length_m')}",
        file=sys.stderr,
    )
    print(f"last pose: {last.get('pose')}", file=sys.stderr)
    print(f"last distance: {last.get('distance')}", file=sys.stderr)


def _stop_scene(base: str) -> None:
    try:
        _post(base, "/api/world/stop")
    except SystemExit as error:
        print(f"stop failed: {error}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--instruction", required=True)
    parser.add_argument(
        "--timeout", type=float, default=120.0,
        help="seconds to keep polling after the task is accepted (default 120)",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a finite positive number")
    base = f"http://{args.host}:{args.port}"
    try:
        _prepare(base)
        _fly(base, args.instruction, args.timeout)
    finally:
        _stop_scene(base)
    return 0


if __name__ == "__main__":
    sys.exit(main())
