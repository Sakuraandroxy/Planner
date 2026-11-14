from dataclasses import dataclass


@dataclass(frozen=True)
class AirSimSettings:
    host: str
    port: int
    camera_id: str
    connect_timeout_s: float
    move_timeout_s: float
    speed_mps: float

