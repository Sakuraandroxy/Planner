"""ProjectAirSim adapters for the VLN skeleton."""

from __future__ import annotations

__all__ = [
    "CameraTopicStream",
    "CollisionMonitor",
    "ProjectAirSimConnection",
    "ProjectAirSimConnectionConfig",
    "ProjectAirSimObservationSource",
    "ProjectAirSimRover",
]


def __getattr__(name: str):
    # Lazy exports keep the pure geometry/config modules importable without
    # the ProjectAirSim client package.
    if name in ("ProjectAirSimConnection", "ProjectAirSimConnectionConfig"):
        from .connection import (
            ProjectAirSimConnection,
            ProjectAirSimConnectionConfig,
        )

        return {
            "ProjectAirSimConnection": ProjectAirSimConnection,
            "ProjectAirSimConnectionConfig": ProjectAirSimConnectionConfig,
        }[name]
    if name == "ProjectAirSimObservationSource":
        from .observation import ProjectAirSimObservationSource

        return ProjectAirSimObservationSource
    if name == "ProjectAirSimRover":
        from .rover import ProjectAirSimRover

        return ProjectAirSimRover
    if name == "CameraTopicStream":
        from .frames import CameraTopicStream

        return CameraTopicStream
    if name == "CollisionMonitor":
        from .collision import CollisionMonitor

        return CollisionMonitor
    raise AttributeError(name)