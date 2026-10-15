from __future__ import annotations

import argparse
import logging
import math
from pathlib import Path

from config import load_config
from planner.bootstrap import build_recording_runtime, build_runtime
from planner.domain.result import MissionResult


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
    runtime = build_runtime(config)
    recording = None
    try:
        logging.info("Connecting to AirSim...")
        runtime.connection.connect()
        runtime.connection.prepare_vehicle()
        if not args.no_takeoff:
            logging.info("Taking off and hovering before task input...")
            runtime.connection.takeoff()
            logging.info("Takeoff and hover completed. Ready for a task.")
        else:
            logging.info("AirSim ready; automatic takeoff skipped.")
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
            return _run(runtime, args.instruction)
        while True:
            instruction = input("\nTask (empty/q to exit): ").strip()
            if not instruction or instruction.lower() in {"q", "quit", "exit"}:
                return 0
            _run(runtime, instruction)
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            if recording is not None:
                recording.close()
        finally:
            runtime.close()


def _run(runtime, instruction: str) -> int:
    try:
        result = runtime.application.execute(instruction)
    except Exception as exc:
        print(f"Task rejected: {exc}")
        return 1
    return _report(result)


def _report(result: MissionResult) -> int:
    for stage in result.stages:
        summary = stage.summary
        if summary is None:
            print(f"Stage {stage.stage_id}: completion not verified; reason={stage.error or 'no summary'}")
            continue
        measurements = "; ".join(f"{item.name}={item.value}{item.unit}" for item in summary.measurements)
        print(f"Stage {stage.stage_id}: {summary.status}; basis={summary.completion_basis.value}; "
              f"reason={summary.reason}" + (f"; {measurements}" if measurements else ""))
        if summary.diagnostics_path:
            print(f"Stage diagnostics: {summary.diagnostics_path}")
    if result.success:
        verified = bool(result.stages) and all(stage.success and stage.summary is not None and
            stage.summary.completion_confirmed for stage in result.stages)
        print(f"Mission completion confirmed by task-specific evidence: {len(result.stages)} stage(s)" if verified
              else f"Stages executed: {len(result.stages)}; task completion not verified")
        return 0
    print(f"Mission failed: {result.error}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

