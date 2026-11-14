"""Explicit assembly of HTTP, lifecycle, worker and read-only heartbeat adapters."""
from pathlib import Path

from planner.adapters.airsim.heartbeat import AirSimHeartbeat
from planner.adapters.http.server_api import create_api
from planner.adapters.runtime.worker import SubprocessPlannerWorker
from planner.application.server_controller import ServerController


def build_server(config, config_path):
    path = Path(config_path).expanduser().resolve()
    controller = ServerController(
        lambda mode: SubprocessPlannerWorker(path, mode),
        lambda: AirSimHeartbeat(config.airsim.host, config.airsim.port, config.server.heartbeat_timeout_s),
        config.server,
        startup_timeout_s=(config.server.readiness_stable_s + config.airsim.connect_timeout_s
                           + 3 * config.airsim.move_timeout_s + 6),
    )
    return create_api(controller, config.server)
