from __future__ import annotations

import argparse
import logging
from pathlib import Path

from config import load_config
from planner.bootstrap import build_recording_runtime, build_runtime
from planner.domain.visual_goal import VisualGoalNavigationResult
from planner.domain.basic_motion import BasicMotionResult


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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("planner").setLevel(logging.INFO)
    config = load_config(args.config)
    runtime = build_runtime(config)
    recording = None
    try:
        runtime.connection.connect()
        runtime.connection.prepare_vehicle()
        if not args.no_takeoff:
            runtime.connection.takeoff()
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
        if recording is not None:
            recording.recorder.stop()
            if recording.recorder.last_error:
                print(f"Camera API recorder stopped after error: {recording.recorder.last_error}")
        runtime.connection.shutdown()


def _run(runtime, instruction: str) -> int:
    try:
        result = runtime.application.execute(instruction)
    except Exception as exc:
        print(f"Task rejected: {exc}")
        return 1
    visual = [stage.value for stage in result.stages if isinstance(stage.value, VisualGoalNavigationResult)]
    basic = [stage.value for stage in result.stages if isinstance(stage.value, BasicMotionResult)]
    for item in basic:
        print(f"Basic motion: {item.status.value}; position_error={item.position_error_m}; "
              f"yaw_error={item.yaw_error_deg}; elapsed={item.elapsed_s:.1f}s; "
              f"stop_confirmed={item.stop.confirmed}; basis={item.completion_basis}; reason={item.reason}")
    for item in visual:
        print(f"Visual goal: {item.status.value}; distance={item.final_distance_m}; "
              f"elapsed={item.elapsed_s:.1f}s; calls={item.planning_calls}; "
              f"stop_confirmed={item.stop.confirmed}; reason={item.reason}")
        if item.run_directory:
            print(f"Visual goal diagnostics: {item.run_directory}")
    if result.success:
        verified = len(basic)+len(visual) == len(result.stages)
        print(f"Mission completion confirmed by task-specific evidence: {len(result.stages)} stage(s)" if verified
              else f"Stages executed: {len(result.stages)}; task completion not verified")
        return 0
    print(f"Mission failed: {result.error}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

