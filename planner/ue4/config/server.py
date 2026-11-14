"""HTTP hosting settings, independent of model and task policy."""
from dataclasses import dataclass, fields

from .validation import integer, mapping, number, string


@dataclass(frozen=True)
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    allowed_origins: tuple[str, ...] = ("*",)
    heartbeat_interval_s: float = 1.0
    heartbeat_timeout_s: float = 1.0
    reconnect_interval_s: float = 2.0
    readiness_stable_s: float = 2.0
    readiness_poll_interval_s: float = 0.25
    log_buffer_size: int = 2000


def server_config(raw) -> ServerConfig:
    mapping(raw, "server", {f.name for f in fields(ServerConfig)})
    values = dict(raw)
    origins = values.get("allowed_origins", ["*"])
    if not isinstance(origins, list) or any(not isinstance(x, str) or not x.strip() for x in origins):
        raise ValueError("server.allowed_origins must be a list of origin strings")
    values["allowed_origins"] = tuple(origins)
    result = ServerConfig(**values)
    string(result.host, "server.host")
    integer(result.port, "server.port", 1, 65535)
    integer(result.log_buffer_size, "server.log_buffer_size", 1, 100000)
    for name in ("heartbeat_interval_s", "heartbeat_timeout_s", "reconnect_interval_s",
                 "readiness_stable_s", "readiness_poll_interval_s"):
        number(getattr(result, name), "server." + name)
    return result
