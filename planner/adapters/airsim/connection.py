from __future__ import annotations

import threading
from typing import Any


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

    def call(self, name: str, *args, **kwargs):
        with self._lock:
            return getattr(self.client, name)(*args, **kwargs)

    def call_async_and_wait(self, name: str, *args, **kwargs):
        with self._lock:
            return getattr(self.client, name)(*args, **kwargs).join()

    def connect(self) -> None:
        self.call("confirmConnection")

    def prepare_vehicle(self) -> None:
        self.call("enableApiControl", True)
        self.call("armDisarm", True)

    def takeoff(self) -> None:
        self.call_async_and_wait("takeoffAsync")
        self.call_async_and_wait("hoverAsync")

    def shutdown(self) -> None:
        try:
            self.call_async_and_wait("hoverAsync")
        except Exception:
            pass
        try:
            self.call("armDisarm", False)
        except Exception:
            pass
        try:
            self.call("enableApiControl", False)
        except Exception:
            pass
