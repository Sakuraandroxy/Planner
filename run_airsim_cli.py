from __future__ import annotations

import argparse
import logging
import math
from pathlib import Path

from config import load_config
from planner.bootstrap import build_recording_runtime
from planner.cli_bootstrap import build_cli_runtime
from planner.adapters.console import read_with_idle
from planner.domain.result import MissionResult
from planner.application.reporting import mission_report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal AirSim trajectory planner")
    parser.add_argument("--config", default=str(Path(__file__).parent / "config" / "base.yaml"))
    parser.add_argument("--instruction", help="run one instruction and exit")
    parser.add_argument("--no-takeoff", action="store_true", help="do not take off automatically")
    parser.add_argument("--record-camera-api", action="store_true", help="save Camera API RGB/depth frames")
    parser.add_argument(
        "--camera-record-mode",
        choices=("frames",),
        default="frames",
        help="recording format; the base planner currently supports frames",
    )
    parser.add_argument("--camera-record-fps", type=float, help="capture rate; defaults to config recording.fps")
    parser.add_argument("--camera-record-output", help="output root; defaults to config recording.output_root")
    args = parser.parse_args()
    if args.camera_record_fps is not None and (not math.isfinite(args.camera_record_fps) or args.camera_record_fps <= 0):
        parser.error("--camera-record-fps must be finite and positive")
    return args


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("planner").setLevel(logging.INFO)
    config = load_config(args.config)
    runtime = None
    recording = None
    try:
        logging.info("Connecting to AirSim...")
        runtime, vehicle = build_cli_runtime(config, takeoff=not args.no_takeoff)
        logging.info("AirSim ready. Waiting for a task.")
        if args.record_camera_api:
            recording = build_recording_runtime(
                config,
                output_root=args.camera_record_output,
                fps=args.camera_record_fps,
            )
            recording.connection.connect()
            output_dir = recording.recorder.start()
            print(f"Camera API recording: {output_dir.resolve()}")
        if args.instruction:
            return _run(runtime, vehicle, args.instruction)
        while True:
            instruction = read_with_idle("\nTask (empty/q to exit): ", vehicle.holding.tick).strip()
            if not instruction or instruction.lower() in {"q", "quit", "exit"}:
                return 0
            _run(runtime, vehicle, instruction)
    except EOFError:
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            if recording is not None:
                recording.close()
        finally:
            if runtime is not None:
                runtime.close()


def _run(runtime, vehicle, instruction: str) -> int:
    try:
        result = runtime.application.execute(instruction)
    except Exception as exc:
        print(f"Task rejected: {exc}")
        return 1
    finally:
        vehicle.hold(require_airborne=False)
    return _report(result)


def _report(result: MissionResult) -> int:
    lines, code = mission_report(result)
    for line in lines:
        print(line)
    return code


if __name__ == "__main__":
    raise SystemExit(main())

