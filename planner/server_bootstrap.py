"""Explicit assembly of the HTTP API, controller, worker and simulator session."""

from __future__ import annotations

from pathlib import Path

from .adapters.http.server_api import create_api
from .simulator import registry
from .simulator.router import FamilyRouter
from .simulator.marker import read_map_marker
from .simulator.worker import ThreadedMissionWorker
from .application.factories import build_detector, build_geometry, build_planner
from .application.mission_assembly import MissionResources
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

    resources = MissionResources(
        detector=detector,
        planner=planner,
        geometry=geometry,
        trajectory=trajectory,
        view_cache=view_cache,
        frame_sink=frames.publish,
    )

    def worker_factory(selection: MapSelection) -> ThreadedMissionWorker:
        # Which simulator owns the map is the catalog's business; the registry turns
        # that into a session, so nothing here names a simulator package.
        entry = config.maps.entry(selection.map_id)
        family = config.families.require(entry.simulator)

        def session_factory(log):
            return registry.build_session(
                entry.simulator,
                config=config,
                map_entry=entry,
                family=family,
                resources=resources,
                log=log,
            )

        return ThreadedMissionWorker(session_factory)

    # One facade for both families. Which driver runs a map follows from the
    # map's family, and both argv templates live in config/simulators.yaml.
    supervisor = FamilyRouter(config.families, config.simulator, PROJECT_DIR, config.maps)
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
