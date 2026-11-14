"""Read-only heartbeat; its client is created and used by the monitor thread."""
from .connection import AirSimConnection
import logging
import time

logger = logging.getLogger(__name__)


class AirSimHeartbeat:
    def __init__(self, host, port, timeout_s, *, connection_factory=AirSimConnection):
        self.host, self.port, self.timeout_s = host, port, timeout_s
        self.factory = connection_factory
        self.connection = None

    def check(self) -> bool:
        began = time.monotonic()
        try:
            if self.connection is None:
                self.connection = self.factory(self.host, self.port, self.timeout_s)
            if self.connection.call("ping") is not True:
                logger.warning("AirSim heartbeat returned false: elapsed=%.3fs", time.monotonic()-began)
                self.close()
                return False
            return True
        except Exception as exc:
            logger.warning('AirSim heartbeat request failed: type=%s elapsed=%.3fs',
                           type(exc).__name__,time.monotonic()-began)
            self.close()
            return False

    def close(self) -> None:
        connection, self.connection = self.connection, None
        if connection is not None:
            try:
                connection.close_transport()
            except Exception:
                pass  # No control or shutdown calls on a read-only transport.
