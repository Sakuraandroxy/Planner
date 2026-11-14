from __future__ import annotations

import threading
import logging
from collections import deque
from typing import Any
from contextlib import contextmanager
from planner.errors import ExecutionError, SimulatorConnectionError
from .rpc_loop import close_rpc_client, configure_rpc_loop

logger = logging.getLogger(__name__)


class AirSimConnection:
    """Serialize state RPCs and optional longer-lived motion RPCs on one owner."""

    def __init__(self, host: str, port: int, timeout_s: float, *, motion_timeout_s: float | None = None):
        try:
            import airsim
        except ImportError as exc:
            raise RuntimeError("AirSim Python package is not installed") from exc
        kwargs: dict[str, Any] = {"port": port, "timeout_value": timeout_s}
        if host:
            kwargs["ip"] = host
        self.client = airsim.MultirotorClient(**kwargs)
        configure_rpc_loop(self.client)
        self._motion_client = None
        self._motion_kwargs = dict(kwargs, timeout_value=motion_timeout_s) if motion_timeout_s is not None else None
        self._lock = threading.RLock()
        self._controls_vehicle = False
        self._closed = False
        self.connection_lost = False
        self._stream_errors = deque()

    def _stream_error(self):
        errors = getattr(self,"_stream_errors",())
        if errors:
            error = errors.popleft()
            if isinstance(error,Exception):
                raise error
            raise ExecutionError(f"AirSim streaming RPC failed: {error}")

    @contextmanager
    def _transport_call(self):
        try:
            yield
        except Exception as exc:
            transport_errors = (OSError, EOFError, TimeoutError)
            try:
                from msgpackrpc.error import TransportError, TimeoutError as RpcTimeoutError
                transport_errors += (TransportError, RpcTimeoutError)
            except ImportError:
                pass
            if isinstance(exc, transport_errors):
                self.connection_lost = True
                raise SimulatorConnectionError(f"AirSim transport failed ({type(exc).__name__})") from exc
            raise

    def call(self, name: str, *args, **kwargs):
        with self._lock:
            if self._closed:
                raise RuntimeError("AirSim connection is closed")
            with self._transport_call():
                result = getattr(self.client, name)(*args, **kwargs)
                self._stream_error()
                return result

    def call_streaming(self, name: str, *args, **kwargs) -> None:
        """Submit on the state transport; owner-side state RPCs pump its I/O.

        New velocity commands replace old ones. False completion of a replaced
        command is normal; RPC errors are collected on the next owner call.
        Callbacks only record errors, and never send flight commands.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError("AirSim connection is closed")
            with self._transport_call():
                self._stream_error()
                future = getattr(self.client,name)(*args,**kwargs)
                def completed(value):
                    if value.error is not None:
                        self._stream_errors.append(value.error)
                future.attach_callback(completed)

    def call_async_and_wait(self, name: str, *args, **kwargs):
        with self._lock:
            if self._closed:
                raise RuntimeError("AirSim connection is closed")
            with self._transport_call():
                client = self.client
                if getattr(self, "_motion_kwargs", None) is not None:
                    if self._motion_client is None:
                        import airsim
                        self._motion_client = airsim.MultirotorClient(**self._motion_kwargs)
                        configure_rpc_loop(self._motion_client)
                    client = self._motion_client
                # msgpackrpc.Future.join() discards both the result and RPC error.
                return getattr(client, name)(*args, **kwargs).get()

    def connect(self) -> None:
        self.call("confirmConnection")

    def prepare_vehicle(self, *, arm: bool = True) -> None:
        if self.call("isApiControlEnabled") is not True:
            logger.info("Requesting AirSim API control.")
            self.call("enableApiControl", True)
            if self.call("isApiControlEnabled") is not True:
                raise ExecutionError("AirSim did not grant API control")
        else:
            logger.info("Reusing enabled AirSim API control without resetting the goal.")
        self._controls_vehicle = True
        if arm and self.call("armDisarm", True) is not True:
            raise ExecutionError("AirSim did not confirm vehicle arming")

    def takeoff(self, *, timeout_s: float | None = None, hover_after: bool = True) -> None:
        options = {} if timeout_s is None else {"timeout_sec": timeout_s}
        if self.call_async_and_wait("takeoffAsync", **options) is not True:
            raise ExecutionError("AirSim takeoff did not complete")
        if hover_after:
            # Native hover follows a path to the current height; a zero-length
            # path may return False. RPC errors still propagate through get().
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

        for name in ("cancelLastTask", "rotateByYawRateAsync"):
            try:
                if name == "rotateByYawRateAsync":
                    self.call_async_and_wait(name, 0., .3)
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
                try:
                    motion = getattr(self, "_motion_client", None)
                    if motion is not None:
                        close_rpc_client(motion)
                finally:
                    close_rpc_client(self.client)
