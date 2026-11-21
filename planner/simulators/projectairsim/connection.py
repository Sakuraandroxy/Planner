"""ProjectAirSim client connection and simulator-clock ownership.

One connection owns the process-wide topic socket (pynng Pair0 is a
single-consumer link) and serializes every service request. Scene files are
loaded through this object and stay paused until the session calls
``resume_clock``, which hands the clock back to the engine.

The clock has two modes. Free-running is the default: the engine renders and
steps on its own, and ``advance_for_sim_time`` only waits until the clock has
moved far enough, so a slow caller never freezes the picture. Stepped is the
fallback for builds that ignore ``resume()`` on a steppable clock; there each
control period is exactly one ``ContinueForSimTime`` request.

Connecting retries with a fresh client because a half-open client can keep the
server's single topic pipe claimed after a failed attempt.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from projectairsim import ProjectAirSimClient, World

from ...errors import SimulatorConnectionError

_PROBE_METHOD = "/Sim/GetBuildCommitHash"

#: Poll interval while waiting on a free-running clock. Each poll is one
#: service request, so the wait below spends most of its time asleep.
_CLOCK_POLL_S = 0.002

#: How long a wait tolerates a clock that refuses to move before it fails. A
#: stalled clock is a broken engine, not a slow one, so the floor is generous.
_CLOCK_STALL_TIMEOUT_S = 2.0

logger = logging.getLogger(__name__)

_ACTIVE_LOCK = threading.Lock()
_ACTIVE_CONNECTION: ProjectAirSimConnection | None = None


@dataclass(frozen=True)
class ProjectAirSimConnectionConfig:
    address: str = "127.0.0.1"
    port_topics: int = 8991
    port_services: int = 8992
    timeout_s: float = 10.0
    scene_file: str = "scene_city_dual.jsonc"
    sim_config_path: str = "config/scenes/city"
    delay_after_load_s: float = 2.0


def resolve_sim_config_path(path: str | Path) -> Path:
    """Resolve a scene-config directory against the repository root."""
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return Path(__file__).resolve().parents[3] / candidate


def _probe_request() -> dict:
    return {"method": _PROBE_METHOD, "params": {}, "version": 1.0}


class ProjectAirSimConnection:
    #: Whether the simulator clock runs by itself. Class-level so a partially
    #: constructed instance (tests build one with ``__new__``) still answers.
    _free_running = False

    def __init__(self, config: ProjectAirSimConnectionConfig):
        self._config = config
        self._lock = threading.RLock()
        self._hooks_lock = threading.Lock()
        self._advance_hooks: list[Callable[[int], None]] = []
        self.client: ProjectAirSimClient | None = None
        self.world: World | None = None
        self._owns_active = False

    # -- lifecycle ---------------------------------------------------------
    def connect(self, *, wait_s: float | None = None) -> None:
        """Open the topic and service links, retrying until ``wait_s``."""
        if self.client is not None:
            raise RuntimeError("connection is already open")
        self._claim_active()
        deadline = time.monotonic() + (
            self._config.timeout_s if wait_s is None else float(wait_s)
        )
        last_error: Exception | None = None
        try:
            while True:
                client = ProjectAirSimClient(
                    self._config.address,
                    self._config.port_topics,
                    self._config.port_services,
                )
                try:
                    client.connect()
                    client.socket_services.recv_timeout = int(
                        max(1.0, self._config.timeout_s) * 1000
                    )
                    client.request(_probe_request())
                except Exception as exc:
                    last_error = exc
                    try:
                        client.disconnect()
                    except Exception:
                        pass
                else:
                    self.client = client
                    return
                if time.monotonic() >= deadline:
                    raise SimulatorConnectionError(
                        f"ProjectAirSim server at {self._config.address}:"
                        f"{self._config.port_topics} did not become ready: "
                        f"{last_error}"
                    ) from last_error
                time.sleep(0.5)
        finally:
            if self.client is None:
                self._release_active()

    def load_scene(
        self,
        scene_file: str | None = None,
        *,
        delay_after_load_s: float | None = None,
    ) -> World:
        """Load a scene file and pause its steppable clock."""
        client = self._require_client()
        name = scene_file or self._config.scene_file
        delay = (
            self._config.delay_after_load_s
            if delay_after_load_s is None
            else float(delay_after_load_s)
        )
        with self._lock:
            # Scene loads exceed the normal request timeout; the delay also
            # gives the renderer time to publish its first sensor topics.
            previous_timeout = client.socket_services.recv_timeout
            client.socket_services.recv_timeout = int(
                max(60.0, self._config.timeout_s) * 1000
            )
            try:
                world = World(
                    client,
                    name,
                    delay_after_load_sec=delay,
                    sim_config_path=str(
                        resolve_sim_config_path(self._config.sim_config_path)
                    ),
                )
                world.pause()
            finally:
                client.socket_services.recv_timeout = previous_timeout
            self.world = world
            return world

    def resume_clock(self, *, verify_s: float = 1.0) -> bool:
        """Unpause the clock and confirm the engine advances it.

        A free-running clock decouples the picture from the control loop: the
        engine keeps rendering while the caller is busy, which a stepped clock
        cannot do. Some builds ignore ``resume()`` on a steppable clock, so the
        mode is confirmed by measurement and falls back to stepping.
        """
        with self._lock:
            world = self._require_world_locked()
            started = int(world.get_sim_time())
            world.resume()
            deadline = time.monotonic() + max(0.1, float(verify_s))
            while time.monotonic() < deadline:
                time.sleep(_CLOCK_POLL_S)
                if int(world.get_sim_time()) > started:
                    self._free_running = True
                    logger.info(
                        "ProjectAirSim clock runs free: a slow caller no longer "
                        "freezes the picture"
                    )
                    return True
            world.pause()
            self._free_running = False
            logger.warning(
                "ProjectAirSim clock did not advance after resume; keeping the "
                "stepped clock, so the picture only moves when a control period "
                "asks for it"
            )
            return False

    def close(self) -> None:
        with self._lock:
            client = self.client
            self.client = None
            self.world = None
        try:
            if client is not None:
                client.disconnect()
        except Exception:
            pass
        finally:
            self._release_active()

    # -- clock -------------------------------------------------------------
    def add_advance_hook(self, hook: Callable[[int], None]) -> None:
        """Register a callback run after every simulator-time advance.

        The callback receives the new simulator time in nanoseconds. It may
        issue service requests: the stepped path holds the re-entrant lock and
        the free-running path waits outside it. Exceptions are logged and never
        break the advance.
        """
        with self._hooks_lock:
            self._advance_hooks.append(hook)

    def remove_advance_hook(self, hook: Callable[[int], None]) -> None:
        with self._hooks_lock:
            self._advance_hooks = [
                registered
                for registered in self._advance_hooks
                if registered is not hook
            ]

    def advance_for_sim_time(self, delta_s: float) -> int:
        """Let the clock move on by ``delta_s`` and return the new time."""
        if (
            isinstance(delta_s, bool)
            or not isinstance(delta_s, (int, float))
            or not math.isfinite(delta_s)
            or delta_s <= 0
        ):
            raise ValueError("simulator advance must be a positive finite time")
        if self._free_running:
            return self._await_clock(float(delta_s))
        nanoseconds = round(float(delta_s) * 1e9)
        with self._lock:
            world = self._require_world_locked()
            result = world.continue_for_sim_time(
                nanoseconds, wait_until_complete=True
            )
            if isinstance(result, int) and not isinstance(result, bool):
                sim_ns = result
            else:
                sim_ns = int(world.get_sim_time())
            self._run_hooks(sim_ns)
            return sim_ns

    def _await_clock(self, delta_s: float) -> int:
        """Wait until a free-running clock has moved on by ``delta_s``.

        The engine steps in real time, so the wait is mostly one sleep and one
        confirmation request. The lock is released while sleeping: holding it
        would serialize the control loop against the frame reader for the whole
        period, which is the coupling this mode exists to remove.
        """
        started = self.sim_time_ns()
        target = started + round(delta_s * 1e9)
        deadline = time.monotonic() + max(_CLOCK_STALL_TIMEOUT_S, delta_s * 4.0)
        coarse = delta_s - _CLOCK_POLL_S
        if coarse > 0:
            time.sleep(coarse)
        while True:
            now = self.sim_time_ns()
            if now >= target:
                self._run_hooks(now)
                return now
            if time.monotonic() >= deadline:
                raise SimulatorConnectionError(
                    "simulator clock did not advance by "
                    f"{delta_s:.3f}s while free-running (stalled at {now} ns)"
                )
            time.sleep(_CLOCK_POLL_S)

    def _run_hooks(self, sim_ns: int) -> None:
        with self._hooks_lock:
            hooks = tuple(self._advance_hooks)
        for hook in hooks:
            try:
                hook(sim_ns)
            except Exception:
                logger.exception("simulator advance hook failed")

    def sim_time_ns(self) -> int:
        with self._lock:
            return int(self._require_world_locked().get_sim_time())

    # -- services ----------------------------------------------------------
    def request(self, method: str, params: dict | None = None) -> Any:
        """Send one service request; callers pass robot-relative method paths."""
        with self._lock:
            client = self._require_client()
            return client.request(
                {"method": method, "params": params or {}, "version": 1.0}
            )

    def scene_configuration(self) -> dict:
        with self._lock:
            return dict(self._require_world_locked().get_configuration())

    def get_time_of_day(self) -> str:
        return str(self._call_world("get_time_of_day"))

    # -- weather / view ----------------------------------------------------
    def enable_weather_effects(self) -> bool:
        return bool(self._call_world("enable_weather_visual_effects"))

    def disable_weather_effects(self) -> bool:
        return bool(self._call_world("disable_weather_visual_effects"))

    def reset_weather_effects(self) -> bool:
        return bool(self._call_world("reset_weather_effects"))

    def set_weather_parameter(self, parameter: int, value: float) -> bool:
        return bool(
            self._call_world("set_weather_visual_effects_param", parameter, value)
        )

    def get_weather_parameters(self) -> dict[int, float]:
        return dict(self._call_world("get_weather_visual_effects_param"))

    def switch_streaming_view(self) -> bool:
        return bool(self._call_world("switch_streaming_view"))

    # -- internals ---------------------------------------------------------
    def _call_world(self, name: str, *args, **kwargs):
        with self._lock:
            return getattr(self._require_world_locked(), name)(*args, **kwargs)

    def _require_client(self) -> ProjectAirSimClient:
        if self.client is None:
            raise RuntimeError("ProjectAirSim connection is closed")
        return self.client

    def _require_world_locked(self) -> World:
        if self.world is None:
            raise RuntimeError("no ProjectAirSim scene has been loaded")
        return self.world

    def _claim_active(self) -> None:
        global _ACTIVE_CONNECTION
        with _ACTIVE_LOCK:
            if _ACTIVE_CONNECTION is not None and _ACTIVE_CONNECTION is not self:
                raise RuntimeError(
                    "another ProjectAirSim connection is active in this process; "
                    "the topic socket is a single-consumer link"
                )
            _ACTIVE_CONNECTION = self
            self._owns_active = True

    def _release_active(self) -> None:
        global _ACTIVE_CONNECTION
        with _ACTIVE_LOCK:
            if self._owns_active and _ACTIVE_CONNECTION is self:
                _ACTIVE_CONNECTION = None
            self._owns_active = False