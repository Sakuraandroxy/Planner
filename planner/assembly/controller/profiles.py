"""Navigation profiles: listing and switching.

The selected profile decides which config the next session is assembled from.
The simulator process keeps running; only the session is rebuilt.
"""

from __future__ import annotations

from ...config import ProfileInfo
from ...domain import BackendUnavailable, ServerState


class ProfileSwitcher:
    def __init__(self, controller):
        self._c = controller

    @property
    def profile(self) -> str:
        """The profile the next session will be built from.

        The assembly reads this when it rebuilds, which is how selecting a
        profile reaches both the in-process session and the UE4 child.
        """
        c = self._c
        with c._lock:
            return c._profile

    @property
    def profiles(self) -> tuple[ProfileInfo, ...]:
        return self._c._profiles

    def request_profile(self, profile: str) -> ServerState:
        """Select a navigation profile.

        With a session running, it is rebuilt from the new file: the profile's values
        are constructor arguments of that session's vehicles and task pipelines, so
        reloading the scene is the only way in. Reloading also puts every vehicle back
        at its spawn, which is what makes two profiles comparable.

        With no session, only the name is recorded; the next scene start reads it.
        """
        c = self._c
        if not c._profiles:
            raise ValueError(
                "this deployment does not list any navigation profiles"
            )
        chosen = str(profile).strip()
        if not any(info.id == chosen for info in c._profiles):
            raise ValueError(
                f"unknown profile '{chosen}'; expected one of "
                f"{[info.id for info in c._profiles]}"
            )
        with c._lock:
            if c._closing.is_set():
                raise BackendUnavailable(c._state)
            if chosen == c._profile:
                return c._state
            if c._state is ServerState.DISCONNECTED:
                c._profile = chosen
                c.lifecycle.log(
                    f"Navigation profile '{chosen}' is selected; it is used when a "
                    "scene starts."
                )
                return c._state
            if (
                c._state is not ServerState.IDLE
                or c._worker is None
                or not c._worker_ready
            ):
                raise BackendUnavailable(c._state)
            c._profile = chosen
            c._profile_switching = True
            c._sim_error = None
            c._active = None
            c._instruction = ""
            c._last_result = None
            c._pending_activation = None
            c._view_pending = False
            c._state = ServerState.CONNECTING
            c._commands.put(("profile", c._generation, chosen))
        c.lifecycle.log(
            f"Switching the navigation profile to '{chosen}'; rebuilding the session."
        )
        return ServerState.CONNECTING

    def switch_profile(self) -> None:
        """Worker CLOSED -> rebuild the session from the profile now selected."""
        c = self._c
        if not c.commands.close_worker(skip_landing=True):
            c.commands.failed(
                "The worker did not confirm shutdown for the profile switch. "
                "Retrying the session."
            )
            return
        with c._lock:
            profile = c._profile
            c._profile_switching = False
        c.lifecycle.log(
            f"Navigation profile '{profile}' is active: its planner endpoint and "
            "navigation settings are wired into the new session."
        )
        c.commands.connect()
