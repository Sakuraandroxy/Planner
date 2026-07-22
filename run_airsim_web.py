#!/usr/bin/env python3
"""Default AirSim web entrypoint.

The default runtime is now the fast-slow sliding-window closed loop.  The
implementation lives in ``agent.functions.fast_slow.runtime``; this file only
starts it so the entrypoint does not accumulate control logic.
"""

from __future__ import annotations

from agent.functions.fast_slow.runtime import run_fast_slow_web


def main() -> None:
    run_fast_slow_web()


if __name__ == "__main__":
    main()
