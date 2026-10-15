from __future__ import annotations

import threading
import logging
from typing import Any

logger = logging.getLogger(__name__)


class AirSimConnection:
    """Own one serialized msgpack-rpc transport."""

    def __init__(self, host: str, port: int, timeout_s: float):
        try:
            import airsim
        except ImportError as exc:
            raise RuntimeError("AirSim Python package is not installed") from exc
        kwargs: dict[str, Any] = {"port": port, "timeout_value": timeout_s}
        if host:
            kwargs["ip"] = host
        self.client = airsim.MultirotorClient(**kwargs)
        self._lock = threading.RLock()
        self._controls_vehicle = False
        self._closed = False

    def call(self, name: str, *args, **kwargs):
        with self._lock:
            if self._closed:
                raise RuntimeError("AirSim connection is closed")
            return getattr(self.client, name)(*args, **kwargs)

    def call_async_and_wait(self, name: str, *args, **kwargs):
        with self._lock:
            if self._closed:
                raise RuntimeError("AirSim connection is closed")
            return getattr(self.client, name)(*args, **kwargs).join()

    def connect(self) -> None:
        self.call("confirmConnection")

    def prepare_vehicle(self) -> None:
        self.call("enableApiControl", True)
        self._controls_vehicle = True
        self.call("armDisarm", True)

    def takeoff(self) -> None:
        self.call_async_and_wait("takeoffAsync")
        self.call_async_and_wait("hoverAsync")

    def shutdown(self) -> None:
        """Hold airborne vehicles; disarm/release only after positive landed evidence."""
        with self._lock:
            if self._closed:
                return
            try:
                if self._controls_vehicle:
                    self._safe_vehicle_shutdown()
            finally:
                self.close_transport()

    def _safe_vehicle_shutdown(self) -> None:
        import airsim

        for name in ("cancelLastTask", "hoverAsync"):
            try:
                if name == "hoverAsync":
                    self.call_async_and_wait(name)
                else:
                    self.call(name)
            except Exception:
                logger.exception("AirSim shutdown could not %s", name)
        try:
            state = self.call("getMultirotorState")
            status = state.landed_state
            landed = (isinstance(status, int) and not isinstance(status, bool)
                      and status == airsim.LandedState.Landed)
        except Exception:
            logger.exception("Cannot confirm landed state; keeping motors/API control unchanged")
            return
        if not landed:
            logger.warning("Vehicle is airborne: requested hover; motors/API control remain enabled")
            return
        try:
            if self.call("armDisarm", False) is not True:
                logger.warning("Disarm was not confirmed; retaining API control")
                return
            self.call("enableApiControl", False)
            self._controls_vehicle = False
        except Exception:
            logger.exception("AirSim landed cleanup failed")

    def close_transport(self) -> None:
        """Close only the RPC transport; safe for read-only recording connections."""
        with self._lock:
            if not self._closed:
                self._closed = True
                self.client.client.close()
