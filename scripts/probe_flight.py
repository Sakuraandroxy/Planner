#!/usr/bin/env python3
"""Fly one instruction through the planner API and record what the stage does.

    python3 scripts/probe_flight.py --instruction "飞到左前方那辆白色汽车旁边" \
        --timeout 700 > run.jsonl

Standard library only, so it runs in either interpreter on the target machine and
needs no planner import. The flow is the console's: read ``/api/status``, start
the scene through ``/api/world`` if none is up, wait for it to become ``idle``,
switch the profile to ``vlm``, submit the task through ``/api/tasks``, poll
``/api/status`` and ``/api/trajectory`` once a second, then stop the task and the
scene it started.

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

Exit codes, because whether a run is worth keeping cannot be read from the JSONL
alone:

===  =====================================================================
0    every goal stage of the task reported success
1    the task ended without reaching its goal, or reported no goal stage
2    the probe's own ``--timeout`` cut the run short; the task was then stopped
3    the probe could not drive the API (HTTP error, scene never became idle)
4    the scene this probe started could not be stopped
===  =====================================================================

A scene the probe did not start is left running: the operator's own scene is not
the probe's to close.
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

EXIT_OK = 0
EXIT_NOT_REACHED = 1
EXIT_TRUNCATED = 2
EXIT_PROBE_ERROR = 3
EXIT_SCENE_STOP_FAILED = 4


class ProbeError(RuntimeError):
    """The API refused a step the probe cannot continue without."""


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
        raise ProbeError(f"{method} {path} answered {error.code}: {detail}")
    except urllib.error.URLError as error:
        raise ProbeError(f"{method} {path} failed: {error.reason}")
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
            raise ProbeError(f"the simulator failed: {status['sim_error']}")
        if time.monotonic() >= deadline:
            raise ProbeError(
                f"the scene did not become idle in {READY_TIMEOUT_S:g}s "
                f"(state={status.get('state')})"
            )
        time.sleep(POLL_INTERVAL_S)


def _prepare(base: str) -> bool:
    """Bring the scene up and select the probe's profile; say whether it was ours."""
    status = _get(base, "/api/status")
    started_scene = False
    if status.get("state") != "idle":
        options = _get(base, "/api/world/options")
        selected = options.get("selection") or {}
        map_id = selected.get("map") or options.get("default_map")
        if not map_id:
            raise ProbeError("no map is selected and the catalog names no default")
        print(f"starting the scene {map_id}", file=sys.stderr)
        _post(base, "/api/world", {"map": map_id})
        status = _wait_ready(base)
        started_scene = True
    if status.get("profile") != "vlm":
        print("switching the profile to vlm", file=sys.stderr)
        _post(base, "/api/profile", {"profile": "vlm"})
        status = _wait_ready(base)
    return started_scene


def _stop_task(base: str) -> None:
    """End the task this probe started and wait until the controller is idle.

    ``/api/world/stop`` only accepts ``idle``, so a run cut short by the probe's
    own deadline has to stop its task first: leaving it running makes the scene
    stop answer 409 and the scene keeps flying after the probe exits.
    """
    print("probe deadline reached; stopping the task", file=sys.stderr)
    _post(base, "/api/stop")
    deadline = time.monotonic() + READY_TIMEOUT_S
    while True:
        status = _get(base, "/api/status")
        if status.get("state") == "idle":
            return
        if time.monotonic() >= deadline:
            raise ProbeError(
                f"the task did not stop in {READY_TIMEOUT_S:g}s "
                f"(state={status.get('state')})"
            )
        time.sleep(POLL_INTERVAL_S)


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


def _fly(base: str, instruction: str, timeout_s: float) -> tuple[list[dict], dict, bool]:
    """Submit the task and record it once a second until it ends or time is up."""
    accepted = _post(base, "/api/tasks", {"instruction": instruction})
    print(
        f"accepted {accepted.get('task_id')} for {accepted.get('vehicle')}",
        file=sys.stderr,
    )
    started = time.monotonic()
    records: list[dict] = []
    snapshot: dict = {}
    truncated = False
    while True:
        status = _get(base, "/api/status")
        snapshot = _get(base, "/api/trajectory")
        record = _sample(time.monotonic() - started, status, snapshot)
        records.append(record)
        print(json.dumps(record, ensure_ascii=False), flush=True)
        # A submitted task runs and then ends in ``stopping`` or ``idle``, both on
        # success and on failure.
        if status.get("state") not in ("running", "stopping"):
            break
        if time.monotonic() - started >= timeout_s:
            truncated = True
            break
        time.sleep(POLL_INTERVAL_S)
    elapsed = time.monotonic() - started
    if truncated:
        _stop_task(base)
    _summary(
        instruction, accepted.get("task_id"), records, snapshot, elapsed, timeout_s
    )
    return records, snapshot, truncated


def _verdict(base: str, truncated: bool) -> int:
    """Read the task's own goal stages; ``last_result`` is prose, not a verdict."""
    if truncated:
        print("reached: no verdict (the probe cut the run short)", file=sys.stderr)
        return EXIT_TRUNCATED
    pending = _get(base, "/api/eval/pending")
    stages = pending.get("goal_stages") or ()
    if not stages:
        print(
            "reached: unknown (the task ended without reporting a goal stage)",
            file=sys.stderr,
        )
        return EXIT_NOT_REACHED
    reached = all(stage.get("success") for stage in stages)
    for stage in stages:
        print(
            f"  {stage.get('stage_id')}: success={stage.get('success')} "
            f"name={stage.get('name')!r}",
            file=sys.stderr,
        )
    print(f"reached: {'yes' if reached else 'no'}", file=sys.stderr)
    return EXIT_OK if reached else EXIT_NOT_REACHED


def _summary(instruction: str, task_id, records: list[dict], snapshot: dict,
             elapsed_s: float, timeout_s: float) -> None:
    last = records[-1] if records else {}
    print("--- summary ---", file=sys.stderr)
    print(f"instruction: {instruction}", file=sys.stderr)
    print(f"task: {task_id}", file=sys.stderr)
    if elapsed_s >= timeout_s and last.get("state") in ("running", "stopping"):
        # The probe's own limit, not the task's: the task budget is 600 s.
        print(
            f"ended: probe deadline ({timeout_s:g}s) cut the run short, "
            f"state={last.get('state')}",
            file=sys.stderr,
        )
    else:
        print(
            f"ended: state={last.get('state')} status={last.get('status')!r} "
            f"after {elapsed_s:.1f}s",
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


def _stop_scene(base: str) -> bool:
    """Stop the scene this probe started; a 409 here is a failed run, not noise."""
    try:
        _post(base, "/api/world/stop")
    except ProbeError as error:
        print(f"stopping the scene failed: {error}", file=sys.stderr)
        return False
    print("scene stopped", file=sys.stderr)
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--instruction", required=True)
    parser.add_argument(
        "--timeout", type=float, default=700.0,
        help="seconds to keep polling after the task is accepted (default 700, "
             "above the 600 s task budget)",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a finite positive number")
    base = f"http://{args.host}:{args.port}"
    started_scene = False
    code = EXIT_OK
    try:
        started_scene = _prepare(base)
        _records, _snapshot, truncated = _fly(base, args.instruction, args.timeout)
        code = _verdict(base, truncated)
    except ProbeError as error:
        print(f"probe failed: {error}", file=sys.stderr)
        code = EXIT_PROBE_ERROR
    finally:
        if started_scene and not _stop_scene(base):
            # A scene left running after the probe exits is a failed run whatever
            # the flight did, so it has to move the exit code.
            code = code or EXIT_SCENE_STOP_FAILED
        elif not started_scene:
            print(
                "the scene was already running; leaving it up", file=sys.stderr
            )
    return code


if __name__ == "__main__":
    sys.exit(main())
