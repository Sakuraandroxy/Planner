#!/usr/bin/env python3
"""M5 runner: the planner HTTP service and console backed by ProjectAirSim.

Start the backend and console with one command:
    conda activate vln
    python scripts/serve_planner.py
The service starts disconnected: no scene is loaded and no simulator socket is
opened until a map is chosen in the console (or POST /api/world). A simulator left
over from a previous run is stopped at startup only when this project started it:
ownership is proved from the engine binary path and the xpra session directory, and
a stack somebody else launched is left alone. Choosing a map brings the stack up
through planner/simulator/xpra_start.py, the one process driver for both families.

Endpoints (see README):
    GET  /              console page
    POST /api/tasks     {"instruction": "...", "vehicle": "rover"|"drone"}
    GET  /api/status
    GET  /api/logs?cursor=N
    GET  /api/trajectory
    GET  /api/video     MJPEG stream (live while running, frozen at idle)
    POST /api/stop
    POST /api/home      {"vehicle": "rover"|"drone"}; idle only, default current vehicle

The worker connects to ProjectAirSim on its own thread; the HTTP process
stays responsive while it waits or while a task runs.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import uvicorn

from planner.config import default_profile, load_config, profile_config_path
from planner.server_bootstrap import build_server


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
    parser.add_argument(
        "--config",
        help="path to a profile's base.yaml; defaults to the --profile directory",
    )
    parser.add_argument(
        "--profile",
        help=f"installed profile to start on (default: {default_profile()})",
    )
    parser.add_argument("--host", help="override server.host")
    parser.add_argument("--port", type=int, help="override server.port")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        config = load_config(
            args.config or str(profile_config_path(args.profile or default_profile()))
        )
        app = build_server(config)
    except (ValueError, OSError) as error:
        raise SystemExit(f"cannot start the planner: {error}") from error

    host = args.host or config.server.host
    port = args.port or config.server.port
    print(
        f"ProjectAirSim VLN planner on http://{host}:{port} "
        f"(console http://{host}:{port}/, "
        f"profile={config.profile.id}, "
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