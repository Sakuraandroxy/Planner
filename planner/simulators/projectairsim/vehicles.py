"""Both actors of a ProjectAirSim session and the parking between them.

A session runs one vehicle at a time, but the scene file declares both robots
and ProjectAirSim has no runtime despawn API, so the inactive one is "hidden"
by a far-away teleport and pinned there as the clock runs. Parking deliberately
does not land the drone: a native land holds one RPC open until touchdown, and
the client closes the whole connection when that reply outruns the request
timeout. A horizontal teleport used to drop the drone inside the Neighbourhood
wall mesh, where it generated thousands of contacts per minute, so the hide spot
is straight up.

``activate_vehicle`` is the one atomic operation behind switching vehicles: the
outgoing one is stopped and teleported out of view, the requested one is homed
onto its spawn and shown through its own chase camera. No step awaits a native
flight command.

The session owns the attributes read here -- ``_rover``, ``_drone``,
``_active_vehicle``, the hidden-pose bookkeeping and ``_log`` -- and this module
owns every transition between them.
"""

from __future__ import annotations

from collections.abc import Callable

from planner.domain import WorldPose
from planner.domain import default_view_for
from ...errors import ExecutionError, ProtocolError
from .drone import ProjectAirSimDrone
from .rover import ProjectAirSimRover

# The inactive vehicle is "hidden" by lifting it this far above where it was
# parked. Robots come from the scene file and cannot be despawned at runtime,
# so a far-away teleport is how only one vehicle stays in view. The lift is
# vertical and large enough to clear every building in both maps: a horizontal
# 300 m teleport put the drone inside the Neighbourhood wall mesh, where the
# physics solver reported ~700 contacts per minute for hours.
HIDE_ALTITUDE_GAIN_M = 1000.0
# The stepped world still applies gravity to the hidden actor, so the pose is
# re-applied at this simulator-time interval (it falls ~20 m between pins,
# which is irrelevant at 1 km). Zero extra cost when nothing is hidden.
HIDE_PIN_PERIOD_S = 2.0


class VehicleRoster:
    """The two actors, the active one, and the hide/park state around them."""

    _rover: ProjectAirSimRover | None
    _drone: ProjectAirSimDrone | None
    _active_vehicle: str
    _log: Callable[..., None]
    # Parked pose of each vehicle currently teleported out of view, and the
    # high-altitude pose it is pinned at while the world steps.
    _hidden_poses: dict[str, WorldPose]
    _hidden_teleports: dict[str, WorldPose]
    _last_pin_ns: int | None
    _pin_failures: set[str]
    _pin_hook: Callable[[int], None]
    # The assembled arms decide which vehicle ids are known; only membership is
    # read here.
    _arms: dict

    def _init_vehicle_state(self) -> None:
        """Start with nothing hidden and no altitude pin taken yet."""
        self._hidden_poses = {}
        self._hidden_teleports = {}
        self._last_pin_ns = None
        self._pin_failures = set()
        # Keep one bound-method object so add/remove refer to the same hook.
        self._pin_hook = self._pin_hidden

    # -- console ports -----------------------------------------------------
    @property
    def active_vehicle(self) -> str:
        return self._active_vehicle

    def return_home(self, vehicle: str | None = None) -> WorldPose:
        """Teleport the requested vehicle to its measured spawn pose and return it."""
        if self._rover is None or self._drone is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        vehicle_id = vehicle or self._active_vehicle
        if vehicle_id not in self._arms:
            raise ProtocolError(f"unknown vehicle '{vehicle_id}'")
        if vehicle_id == "drone":
            pose = self._drone.return_home()
            self._log(
                "Drone returned to start position: "
                f"x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} "
                f"yaw={pose.yaw_deg:.1f}"
            )
        else:
            pose = self._rover.return_home()
            self._log(
                "Rover returned to start position: "
                f"x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} "
                f"yaw={pose.yaw_deg:.1f}"
            )
        # The vehicle is at its spawn again, so any hidden-pose record is stale.
        self._hidden_poses.pop(vehicle_id, None)
        self._hidden_teleports.pop(vehicle_id, None)
        if vehicle_id != self._active_vehicle:
            # An explicit home request for the inactive vehicle must not
            # bring it back into view; only the active vehicle is shown.
            self._hide(vehicle_id)
        return pose

    def activate_vehicle(self, vehicle: str | None = None) -> str:
        """Console switch: stop the other vehicle, home and show this one.

        This is the one atomic operation behind clicking a vehicle in the
        console. Each side moves once: the outgoing vehicle stops flying or
        holds its brake and is teleported out of view and pinned there, and the
        requested vehicle is teleported straight onto its spawn. No step awaits
        a native flight command.
        """
        if self._rover is None or self._drone is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        vehicle_id = vehicle or self._active_vehicle
        if vehicle_id not in self._arms:
            raise ProtocolError(f"unknown vehicle '{vehicle_id}'")
        other = "rover" if vehicle_id == "drone" else "drone"
        if not self._park(other, self._log):
            raise ExecutionError(
                f"could not park the {other}; {vehicle_id} was not activated"
            )
        try:
            self._hide(other)
        except Exception as error:
            raise ExecutionError(
                f"could not hide the {other}; {vehicle_id} was not activated"
            ) from error
        # The parked pose is not restored first: a console switch homes the
        # vehicle anyway.
        self._recover(vehicle_id)
        previous_active = self._active_vehicle
        # Mark the target active before homing it so ``return_home`` does not
        # immediately hide it again as an inactive vehicle.
        self._active_vehicle = vehicle_id
        try:
            self.return_home(vehicle_id)
            view = default_view_for(vehicle_id)
            self.set_view_mode(view)
        except Exception:
            self._active_vehicle = previous_active
            if vehicle_id != previous_active:
                try:
                    self._hide(vehicle_id)
                except Exception:
                    pass
            raise
        self._log(
            f"Vehicle activated: {vehicle_id} "
            f"(parked {other}, hid it out of view, returned home, view={view})."
        )
        return vehicle_id

    # -- parking -----------------------------------------------------------
    def _vehicle_for(self, vehicle_id: str):
        vehicle = self._drone if vehicle_id == "drone" else self._rover
        if vehicle is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        return vehicle

    def _park(self, vehicle_id: str, log) -> bool:
        """Make an inactive vehicle inert so it can be moved out of view.

        Parking is not landing: the drone stops flying, the rover holds its
        brake, and the caller teleports that vehicle above the map right after
        this returns. Returns False only when the vehicle could not be stopped.
        """
        try:
            if vehicle_id == "drone" and self._drone is not None:
                self._drone.park()
                log("Drone parked: flight stopped.")
            elif self._rover is not None:
                self._rover.hold()
            return True
        except Exception as error:
            log(
                f"parking the {vehicle_id} failed: "
                f"{type(error).__name__}: {error}"
            )
            return False

    def _hide_inactive_at_startup(self) -> None:
        """Move the non-default vehicle out of view right after loading."""
        self._hide_quiet("drone" if self._active_vehicle == "rover" else "rover")

    def _hide_quiet(self, vehicle_id: str) -> None:
        """Move one vehicle out of view; a refused teleport is a boot warning."""
        try:
            self._hide(vehicle_id)
        except Exception as error:
            self._log(
                f"could not move the {vehicle_id} out of view: "
                f"{type(error).__name__}: {error}",
                "WARNING",
            )

    def _unhide_quiet(self, vehicle_id: str) -> None:
        """Restore one vehicle; a refused teleport is a boot warning."""
        try:
            self._unhide(vehicle_id)
        except Exception as error:
            self._log(
                f"could not bring the {vehicle_id} back into view: "
                f"{type(error).__name__}: {error}",
                "WARNING",
            )

    def _hide(self, vehicle_id: str) -> None:
        """Teleport an inactive vehicle high above the map, out of view.

        ProjectAirSim cannot despawn a robot at runtime (robots are declared
        in the scene file), so hiding is a far-away teleport. The move is
        straight up: a horizontal offset can land inside map geometry, and
        while a hidden actor overlaps a wall the physics solver emits
        contacts continuously. The parked pose is remembered so ``_unhide``
        can put the vehicle back exactly where a task left it, and
        ``_pin_hidden`` keeps it at altitude as the clock runs.
        """
        if vehicle_id in self._hidden_poses:
            return
        vehicle = self._vehicle_for(vehicle_id)
        pose = vehicle.current_pose()
        hidden = WorldPose(
            pose.x,
            pose.y,
            pose.z - HIDE_ALTITUDE_GAIN_M,
            pose.yaw_deg,
        )
        vehicle.teleport(hidden)
        self._hidden_poses[vehicle_id] = pose
        self._hidden_teleports[vehicle_id] = hidden
        self._pin_failures.discard(vehicle_id)
        self._log(
            f"{vehicle_id.capitalize()} moved out of view "
            f"({HIDE_ALTITUDE_GAIN_M:.0f} m up); "
            "only the active vehicle is shown."
        )

    def _unhide(self, vehicle_id: str) -> None:
        """Bring a hidden vehicle back to the pose it was parked at."""
        pose = self._hidden_poses.pop(vehicle_id, None)
        self._hidden_teleports.pop(vehicle_id, None)
        self._pin_failures.discard(vehicle_id)
        if pose is None:
            return
        self._vehicle_for(vehicle_id).teleport(pose)
        self._log(
            f"{vehicle_id.capitalize()} restored to its previous position."
        )

    def _recover(self, vehicle_id: str) -> None:
        """Forget where a vehicle was parked, so the altitude pin lets it move.

        The switch homes the vehicle right after this, so the pose a task would
        resume from is discarded rather than restored; ``_unhide`` is what puts
        it back for a task submission.
        """
        self._hidden_poses.pop(vehicle_id, None)
        self._hidden_teleports.pop(vehicle_id, None)
        self._pin_failures.discard(vehicle_id)

    def _pin_hidden(self, sim_ns: int) -> None:
        """Re-apply the hide pose so gravity cannot pull a hidden actor down.

        Runs after every clock advance (see ``add_advance_hook``). The pin is
        throttled by simulator time; between pins the actor falls only ~20 m,
        which keeps it far above every building in both maps.
        """
        if not self._hidden_teleports:
            return
        period_ns = int(HIDE_PIN_PERIOD_S * 1e9)
        if (
            self._last_pin_ns is not None
            and 0 <= sim_ns - self._last_pin_ns < period_ns
        ):
            return
        self._last_pin_ns = sim_ns
        for vehicle_id, pose in tuple(self._hidden_teleports.items()):
            try:
                self._vehicle_for(vehicle_id).teleport(pose)
            except Exception as error:
                if vehicle_id not in self._pin_failures:
                    self._pin_failures.add(vehicle_id)
                    self._log(
                        f"could not keep the hidden {vehicle_id} at altitude: "
                        f"{type(error).__name__}: {error}",
                        "WARNING",
                    )
            else:
                self._pin_failures.discard(vehicle_id)
