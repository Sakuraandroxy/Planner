"""Explicit assembly of the HTTP API, controller, worker and simulator session."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from .adapters.http.server_api import create_api
from .adapters.runtime.projectairsim_session import ProjectAirSimMissionSession
from .adapters.runtime.simulator_supervisor import (
    SimulatorSupervisor,
    read_map_marker,
)
from .adapters.runtime.worker import ThreadedMissionWorker
from .application.factories import build_detector, build_geometry, build_planner
from .application.server_controller import ServerController
from .config import AppConfig
from .domain.server import MapSelection
from .services.frame_hub import FrameHub
from .services.trajectory_hub import TrajectoryHub

PROJECT_DIR = Path(__file__).resolve().parent.parent


def build_server(config: AppConfig):
    """Assemble the service; it starts disconnected until a scene is chosen."""
    detector = build_detector(
        config,
        config.server.detector,
        bbox=config.server.manual_bbox,
        surface_points=config.server.manual_surface_points,
    )
    planner = build_planner(config, config.server.planner)
    geometry = build_geometry(config)
    frames = FrameHub(max_fps=config.server.video_fps)
    trajectory = TrajectoryHub()
    # Measured streaming-view cycle orders, keyed by scene file. Shared across
    # worker rebuilds so a repeated map/model switch only pays for a cheap
    # one-screenshot anchor check instead of the full calibration walk.
    view_cache: dict[str, tuple[str, ...]] = {}

    def worker_factory(selection: MapSelection) -> ThreadedMissionWorker:
        # The selected catalog entry is the single source of the scene file
        # and its config directory; everything else comes from base.yaml.
        session_config = replace(
            config,
            projectairsim=replace(
                config.projectairsim,
                scene_file=selection.scene_file,
                sim_config_path=selection.sim_config_path,
            ),
        )

        def session_factory(log):
            return ProjectAirSimMissionSession(
                session_config,
                detector=detector,
                planner=planner,
                geometry=geometry,
                log=log,
                frame_sink=frames.publish,
                trajectory=trajectory,
                view_cache=view_cache,
            )

        return ThreadedMissionWorker(session_factory)

    supervisor = SimulatorSupervisor(config.simulator, PROJECT_DIR)
    marker = Path(config.simulator.map_marker_file)
    if not marker.is_absolute():
        marker = PROJECT_DIR / marker
    initial_map = read_map_marker(marker, config.maps) or config.maps.default_map

    controller = ServerController(
        worker_factory,
        config.server,
        maps=config.maps,
        simulator=supervisor,
        initial_map=initial_map,
        vehicle_models=config.vehicle_models,
        restart_attempts=config.simulator.restart_attempts,
        worker_shutdown_timeout_s=config.drone.land_timeout_s + 10.0,
        autostart_session=False,
    )
    return create_api(
        controller,
        config.server,
        frames=frames,
        trajectory_hub=trajectory,
    )
