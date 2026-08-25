#!/usr/bin/env python3
"""Default AirSim web entrypoint.

The default runtime is now the fast-slow sliding-window closed loop.  The
implementation lives in ``agent.functions.fast_slow.runtime``; this file only
starts it so the entrypoint does not accumulate control logic.
"""

from __future__ import annotations

import argparse

from agent.functions.debug import TargetSnapshotRecorder
from agent.functions.fast_slow.runtime import run_fast_slow_web


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Uni-LaViRA AirSim web runtime")
    parser.add_argument(
        "--save-target-snapshots",
        action="store_true",
        help="保存每个实际锁定目标的首次检测框图（默认关闭）",
    )
    args = parser.parse_args(argv)
    recorder = TargetSnapshotRecorder(enabled=args.save_target_snapshots)
    if recorder.enabled:
        print(f"[TargetSnapshot] enabled output={recorder.run_directory}")
    run_fast_slow_web(target_snapshot_recorder=recorder)


if __name__ == "__main__":
    main()
