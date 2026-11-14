#!/usr/bin/env python3
"""M5 runner: the planner HTTP service and console backed by ProjectAirSim.

Start the backend and console with one command:
    conda activate carla-vln
    python scripts/serve_planner.py
The service starts disconnected: no scene is loaded and no simulator socket is
opened until a map is chosen in the console (or POST /api/world). A simulator
left over from a previous run is stopped at startup when the launch script owns
it; an externally started one is left alone. Choosing a map then launches the
stack through scripts/launch_projectairsim_xpra.sh (an unowned stack running the
same map is attached to).

Endpoints (see README):
    GET  /              console page
    POST /api/tasks     {"instruction": "...", "vehicle": "rover"|"drone"}
    GET  /api/status
    GET  /api/logs?cursor=N
    GET  /api/trajectory
    GET  /api/video     MJPEG stream (live while running, frozen at idle)
    POST /api/stop
    POST /api/home      rover only, idle only

The worker connects to ProjectAirSim on its own thread; the HTTP process
stays responsive while it waits or while a task runs.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import uvicorn  # noqa: E402

from planner.config import load_config  # noqa: E402
from planner.server_bootstrap import build_server  # noqa: E402


class PlannerServer(uvicorn.Server):
    """End MJPEG streams before uvicorn waits for connections to drain."""

    def __init__(self, config: uvicorn.Config, frames) -> None:
        super().__init__(config)
        self._frames = frames

    def handle_exit(self, sig, frame) -> None:
        if self._frames is not None:
            self._frames.close()
        super().handle_exit(sig, frame)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config" / "base.yaml"))
    parser.add_argument("--host", help="override server.host")
    parser.add_argument("--port", type=int, help="override server.port")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        config = load_config(args.config)
        app = build_server(config)
    except (ValueError, OSError) as error:
        raise SystemExit(f"cannot start the planner: {error}") from error

    host = args.host or config.server.host
    port = args.port or config.server.port
    print(
        f"ProjectAirSim VLN planner on http://{host}:{port} "
        f"(console http://{host}:{port}/, "
        f"detector={config.server.detector}, "
        f"planner={config.server.planner}, "
        f"scene={config.projectairsim.scene_file})"
    )
    server = PlannerServer(
        uvicorn.Config(app, host=host, port=port, log_level="info"),
        getattr(app.state, "frames", None),
    )
    try:
        server.run()
    except KeyboardInterrupt:
        # uvicorn re-raises the captured SIGINT after a graceful shutdown.
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())