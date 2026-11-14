"""ProjectAirSim client connection and steppable-clock ownership.

One connection owns the process-wide topic socket (pynng Pair0 is a
single-consumer link) and serializes every service request. Scene files are
loaded through this object; the session keeps the world paused and advances it
with ``advance_for_sim_time`` so every control period is exactly one request.

Connecting retries with a fresh client because a half-open client can keep the
server's single topic pipe claimed after a failed attempt.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import logging
import math
import threading
import time
from typing import Any, Callable

from projectairsim import ProjectAirSimClient, World

from ...errors import SimulatorConnectionError

_PROBE_METHOD = "/Sim/GetBuildCommitHash"

logger = logging.getLogger(__name__)

_ACTIVE_LOCK = threading.Lock()
_ACTIVE_CONNECTION: "ProjectAirSimConnection | None" = None


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

        The callback receives the new simulator time in nanoseconds. It runs
        under the connection lock, so it may issue service requests (the lock
        is re-entrant); exceptions are logged and never break the advance.
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
        """Advance the paused clock by ``delta_s`` and return the new time."""
        if (
            isinstance(delta_s, bool)
            or not isinstance(delta_s, (int, float))
            or not math.isfinite(delta_s)
            or delta_s <= 0
        ):
            raise ValueError("simulator advance must be a positive finite time")
        nanoseconds = int(round(float(delta_s) * 1e9))
        with self._lock:
            world = self._require_world_locked()
            result = world.continue_for_sim_time(
                nanoseconds, wait_until_complete=True
            )
            if isinstance(result, int) and not isinstance(result, bool):
                sim_ns = result
            else:
                sim_ns = int(world.get_sim_time())
            with self._hooks_lock:
                hooks = tuple(self._advance_hooks)
            for hook in hooks:
                try:
                    hook(sim_ns)
                except Exception:  # noqa: BLE001
                    logger.exception("simulator advance hook failed")
            return sim_ns

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