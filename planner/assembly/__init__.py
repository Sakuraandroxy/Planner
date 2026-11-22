"""Assembly layer entry.

Members are the names other layers reach for: the session factory, the port
requirement table, the resource builders, the backend facade. Names the layer
uses on its own stay in their modules and are imported from there.
"""

from .controller import ServerController
from .controller.state import (
    BackendUnavailable,
    MapSelection,
    ServerState,
)
from .factories import (
    build_detector,
    build_free_navigation_planner,
    build_geometry,
    build_planner,
)
from .mission_assembly import (
    MissionInputs,
    MissionResources,
    VehicleArm,
    assemble_missions,
    require_application,
)
from .reporting import task_finish_events
from .requirements import capabilities_for

__all__ = [
    "BackendUnavailable",
    "MapSelection",
    "MissionInputs",
    "MissionResources",
    "ServerController",
    "ServerState",
    "VehicleArm",
    "assemble_missions",
    "build_detector",
    "build_free_navigation_planner",
    "build_geometry",
    "build_planner",
    "capabilities_for",
    "require_application",
    "task_finish_events",
]
