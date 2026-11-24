from __future__ import annotations

import inspect
import logging
import math
import threading
import time
from collections import deque
from contextlib import contextmanager
from typing import Any

from planner.errors import ExecutionError, SimulatorConnectionError

from .rpc_loop import close_rpc_client, configure_rpc_loop

logger = logging.getLogger(__name__)

#: What an async motion has to prove to be called done. Blocks measured a
#: takeoffAsync that returned ``False`` while the aircraft climbed 1.54 m, so the
#: ack is logged and height is what gets checked.
MIN_CLIMB_M = 0.5

#: Cache of "does this client method take a vehicle_name?", keyed by __qualname__.
_VEHICLE_ARG: dict[str, bool] = {}

#: How long an abandoned wait gets to unwind before the caller moves on.
_ABANDON_REAP_S = 0.5


def _accepts_vehicle_name(func) -> bool:
    """Whether this AirSim client method answers to a named vehicle.

    The server reads state calls from the default vehicle even when settings.json
    names it ``Drone_1``, so leaving the name blank silently routes control calls
    to a key that does not exist: ``moveToPositionAsync`` returns ``False`` and
    ``landAsync`` raises an RPC exception. Not every method has the parameter
    (``listVehicles`` does not), so the answer comes from the signature instead of
    a hand-kept list that drifts when the client changes.
    """
    key = getattr(func, "__qualname__", "")
    cached = _VEHICLE_ARG.get(key)
    if cached is None:
        try:
            parameters = inspect.signature(func).parameters
        except (TypeError, ValueError):
            cached = False
        else:
            cached = "vehicle_name" in parameters or any(
                p.kind is p.VAR_KEYWORD for p in parameters.values()
            )
        _VEHICLE_ARG[key] = cached
    return cached


class AirSimConnection:
    """Serialize state RPCs and optional longer-lived motion RPCs on one owner."""

    def __init__(self, host: str, port: int, timeout_s: float, *, motion_timeout_s: float | None = None,
                 vehicle_name: str = ""):
        try:
            import airsim
        except ImportError as exc:
            raise RuntimeError("AirSim Python package is not installed") from exc
        kwargs: dict[str, Any] = {"port": port, "timeout_value": timeout_s}
        if host:
            kwargs["ip"] = host
        self.client = airsim.MultirotorClient(**kwargs)
        #: The vehicle every call targets. Empty keeps AirSim's own default.
        self.vehicle_name = vehicle_name
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
                from msgpackrpc.error import TimeoutError as RpcTimeoutError
                from msgpackrpc.error import TransportError
                transport_errors += (TransportError, RpcTimeoutError)
            except ImportError:
                pass
            if isinstance(exc, transport_errors):
                self.connection_lost = True
                raise SimulatorConnectionError(f"AirSim transport failed ({type(exc).__name__})") from exc
            raise

    def _for_vehicle(self, func, kwargs: dict[str, Any]) -> dict[str, Any]:
        """Add the vehicle name unless the method cannot take one or a caller chose."""
        if not self.vehicle_name or "vehicle_name" in kwargs:
            return kwargs
        if not _accepts_vehicle_name(func):
            return kwargs
        return {**kwargs, "vehicle_name": self.vehicle_name}

    def call(self, name: str, *args, **kwargs):
        with self._lock:
            if self._closed:
                raise RuntimeError("AirSim connection is closed")
            func = getattr(self.client, name)
            kwargs = self._for_vehicle(func, kwargs)
            with self._transport_call():
                result = func(*args, **kwargs)
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
            func = getattr(self.client, name)
            kwargs = self._for_vehicle(func, kwargs)
            with self._transport_call():
                self._stream_error()
                future = func(*args, **kwargs)
                def completed(value):
                    if value.error is not None:
                        self._stream_errors.append(value.error)
                future.attach_callback(completed)

    def call_async_and_wait(self, name: str, *args, timeout_s: float | None = None,
                            **kwargs):
        """Issue an async RPC and wait for its ack, optionally with a deadline.

        ``timeout_s`` is the client-side limit, not AirSim's ``timeout_sec``: the
        two are separate keywords and a caller that wants both passes both. The
        SDK's ``Future.get()`` cannot be bounded -- msgpackrpc's ``join`` is
        ``while not flag: loop.start()``, a pump loop with no deadline and no
        cancel -- so the deadline waits on a helper thread that does the pumping
        and the caller breaks that wait through the loop (``rpc_loop.py``) when
        the deadline passes. Breaking it is what reclaims the thread: leaving it
        pumping would put a second thread on the same IOLoop, where the loser
        spins and shutdown races the loop it is inside.

        The connection lock is released before waiting: a path call can take a
        minute, and state reads on the owner thread must not queue behind it.
        """
        if timeout_s is not None:
            timeout_s = float(timeout_s)
            if not math.isfinite(timeout_s) or timeout_s <= 0:
                raise ValueError("timeout_s must be a finite positive number")
        with self._transport_call():
            with self._lock:
                if self._closed:
                    raise RuntimeError("AirSim connection is closed")
                client = self.client
                if getattr(self, "_motion_kwargs", None) is not None:
                    if self._motion_client is None:
                        import airsim
                        self._motion_client = airsim.MultirotorClient(**self._motion_kwargs)
                        configure_rpc_loop(self._motion_client)
                    client = self._motion_client
                func = getattr(client, name)
                # msgpackrpc.Future.join() discards both the result and RPC error.
                future = func(*args, **self._for_vehicle(func, kwargs))
                # The loop that pumps this future; the SDK session owns it.
                loop = getattr(getattr(client, "client", None), "_loop", None)
            if timeout_s is None:
                return future.get()
            outcome: dict[str, object] = {}

            def wait():
                try:
                    outcome["value"] = future.get()
                except BaseException as error:  # re-raised by the owner below
                    outcome["error"] = error

            waiter = threading.Thread(
                target=wait, name=f"airsim-wait-{name}", daemon=True
            )
            waiter.start()
            waiter.join(timeout_s)
            if waiter.is_alive():
                abandon = getattr(loop, "abandon", None)
                if abandon is not None:
                    abandon()
                    waiter.join(_ABANDON_REAP_S)
                if waiter.is_alive():
                    logger.warning(
                        "the wait for %s could not be unwound; it ends on the "
                        "client's own RPC timeout",
                        name,
                    )
                raise ExecutionError(
                    f"AirSim {name} did not answer inside {timeout_s:g}s"
                )
            if "error" in outcome:
                raise outcome["error"]
            return outcome.get("value")

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

    def takeoff(self, *, timeout_s: float | None = None, hover_after: bool = True,
                settle_timeout_s: float = 30.0) -> None:
        """Climb, and prove the climb from state rather than from the ack.

        On the target build ``takeoffAsync().get()`` answers ``False`` while the
        aircraft actually rises -- measured 0.68 m to -0.86 m on Blocks -- and
        ``landed_state`` is no substitute in the other direction either: after a
        ``landAsync()`` that returned ``True`` it stayed at ``1`` (Flying) with the
        vehicle back on its spawn height. Height gained is the evidence that does
        not lie, so that is what this waits for.
        """
        options = {} if timeout_s is None else {"timeout_sec": timeout_s}
        before = self._ground_z()
        ack = self.call_async_and_wait("takeoffAsync", **options)
        if ack is not True:
            logger.info("takeoffAsync ack=%r; deciding the takeoff from state", ack)
        deadline = time.monotonic() + settle_timeout_s
        gained = before - self._ground_z()
        while gained < MIN_CLIMB_M and time.monotonic() < deadline:
            time.sleep(0.2)
            gained = before - self._ground_z()
        if gained < MIN_CLIMB_M:
            raise ExecutionError(
                f"AirSim takeoff did not lift the vehicle: gained {gained:.2f} m in "
                f"{settle_timeout_s:g}s (need {MIN_CLIMB_M} m), ack={ack!r}"
            )
        if hover_after:
            # Native hover follows a path to the current height; a zero-length
            # path may return False. RPC errors still propagate through get().
            self.call_async_and_wait("hoverAsync")

    def _ground_z(self) -> float:
        return self.call("getMultirotorState").kinematics_estimated.position.z_val

    def shutdown(self, *, landed_on_ground: bool = False) -> None:
        """Hold airborne vehicles; disarm/release only after positive landed evidence.

        ``landed_on_ground`` is for a caller that measured the touchdown itself.
        This build leaves ``landed_state`` at 1 (Flying) even with the vehicle back on
        its spawn height, so refusing on that flag alone would keep every landed drone
        armed -- and refusing is only right when nothing else proves the landing.
        """
        with self._lock:
            if self._closed:
                return
            try:
                if self._controls_vehicle:
                    self._safe_vehicle_shutdown(landed_on_ground=landed_on_ground)
            finally:
                self.close_transport()

    def _safe_vehicle_shutdown(self, *, landed_on_ground: bool = False) -> None:
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
            landed = landed_on_ground or (
                isinstance(status, int) and not isinstance(status, bool)
                and status == airsim.LandedState.Landed
            )
        except Exception:
            logger.exception("Cannot confirm landed state; keeping motors/API control unchanged")
            return
        if not landed:
            logger.warning(
                "Landing was not confirmed by landed_state (%r) at z=%.2f: requesting "
                "hover and leaving motors/API control enabled. This build keeps "
                "landed_state at 1 (Flying) even after a land that returned True; pass "
                "landed_on_ground with a measured touchdown to disarm safely.",
                status, state.kinematics_estimated.position.z_val,
            )
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
