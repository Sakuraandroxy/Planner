from __future__ import annotations

import argparse
from pathlib import Path

from config import load_config
from planner.bootstrap import build_runtime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal AirSim trajectory planner")
    parser.add_argument("--config", default=str(Path(__file__).parent / "config" / "base.yaml"))
    parser.add_argument("--instruction", help="run one instruction and exit")
    parser.add_argument("--no-takeoff", action="store_true", help="do not take off automatically")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    runtime = build_runtime(load_config(args.config))
    try:
        runtime.connection.connect()
        runtime.connection.prepare_vehicle()
        if not args.no_takeoff:
            runtime.connection.takeoff()
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
        runtime.connection.shutdown()


def _run(runtime, instruction: str) -> int:
    result = runtime.application.execute(instruction)
    if result.success:
        print(f"Mission executed: {len(result.stages)} stage(s)")
        return 0
    print(f"Mission failed: {result.error}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

