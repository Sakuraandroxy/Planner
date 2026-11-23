"""ProjectAirSim Drone adapter: connection, ports, hold and observations.

The drone is a ``fast-physics`` SimpleFlight quadrotor. ``position_hold`` is
the reference owner-thread hold service: every command is a ``MoveByVelocity``
for one control period. The simulator clock runs free after scene load, so a
command does not have to wait for the planner to step the world.

The native takeoff/land/home sequences live in ``flight``, the command
downlink in ``commands``, pose/kinematics conversion in ``coordinates`` and
frame acquisition in ``observation``. This class owns the connection, the
ports the planner drives, and the drone's own state: ground altitude, home
pose, hold and collision latch. Each port method either reads that state or
delegates to the module that implements it.
"""

from __future__ import annotations

import logging
import threading
import time

from projectairsim.types import LandedState

from ...config import CameraSettings, DepthSettings
from planner.domain import Observation
from planner.domain import WorldPose
from planner.domain import TimedVehicleState
from ...errors import ExecutionError
from planner.runtime import PositionHold
from planner.domain import (
    CollisionContact,
    CollisionState,
    MotionCapability,
    MotionCommand,
    MotionLimits,
    MotionModel,
    require_supported,
)
from . import flight
from .async_bridge import AsyncBridge
from .collision import CollisionEvent, CollisionMonitor
from .commands import ProjectAirSimCommands
from .connection import ProjectAirSimConnection
from .coordinates import pose_from_kinematics, state_from_kinematics
from .motion import tracking_limits
from .observation import ProjectAirSimObservationSource

logger = logging.getLogger(__name__)


class ProjectAirSimDrone:
    def __init__(
        self,
        connection: ProjectAirSimConnection,
        robot,
        *,
        camera: CameraSettings,
        depth: DepthSettings = DepthSettings(),
        camera_id: str = "FrontCamera",
        speed_mps: float = 2.0,
        motion_timeout_s: float = 60.0,
        takeoff_timeout_s: float = 30.0,
        land_timeout_s: float = 30.0,
        readiness_stable_s: float = 2.0,
        readiness_poll_interval_s: float = 0.25,
        takeoff_climb_m: float = 0.0,
        motion: MotionLimits | None = None,
        tracking=None,
        frame_sink=None,
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        from planner.domain import ContinuousTrackingSettings

        self._connection = connection
        self._robot = robot
        self._speed_mps = float(speed_mps)
        self._motion_timeout_s = float(motion_timeout_s)
        self._takeoff_timeout_s = float(takeoff_timeout_s)
        self._land_timeout_s = float(land_timeout_s)
        self._land_settle_timeout_s = 5.0
        self._readiness_stable_s = float(readiness_stable_s)
        self._readiness_poll_interval_s = float(readiness_poll_interval_s)
        self._takeoff_climb_m = float(takeoff_climb_m)
        self._clock = clock
        self._sleep = sleep
        self._tracking = tracking_limits(motion or MotionLimits())
        self._capability = MotionCapability(
            MotionModel.HOLONOMIC,
            max_lateral_speed_mps=self._tracking.max_speed_mps,
            min_turn_radius_m=None,
            allows_reverse=True,
        )
        self._hold_tracking = tracking or ContinuousTrackingSettings()
        self._period = 1.0 / self._tracking.control_hz
        self._cancel = threading.Event()
        self._started = False
        self._bridge = AsyncBridge()
        self._commands = ProjectAirSimCommands(connection, robot, self._bridge)
        self._observations = ProjectAirSimObservationSource(
            connection,
            robot,
            camera=camera,
            depth=depth,
            camera_id=camera_id,
            frame_sink=frame_sink,
            settle=self._settle_capture,
        )
        self._collisions = CollisionMonitor(
            connection.client,
            robot.robot_info["collision_info"],
            ignored_names=(robot.name,),
        )
        self._collisions.start()
        self._ground_z: float | None = None
        # First settled pose captured before the first takeoff; ``return_home``
        # teleports here (instant, matching the rover home semantics).
        self._home_pose: WorldPose | None = None
        self._last_collision: CollisionEvent | None = None
        self._holding = PositionHold(
            self,
            self._tracking,
            self._hold_tracking,
            self._speed_mps,
            self._motion_timeout_s,
            clock=clock,
            sleep=sleep,
        )

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        """Enable API control and arm; required before any flight command."""
        if not self._robot.enable_api_control():
            raise ExecutionError("drone refused API control")
        if not self._robot.arm():
            raise ExecutionError("drone refused arm")
        # Capture the spawn pose before the session teleports an inactive drone
        # out of view; a later capture would record the hidden pose as home.
        if self._home_pose is None:
            self._home_pose = self.current_pose()
        self._started = True

    def prepare(
        self, *, takeoff_if_landed: bool = True, require_airborne: bool = False
    ) -> None:
        """Reference readiness sequence: tolerant native takeoff, then hold."""
        flight.prepare(
            self,
            takeoff_if_landed=takeoff_if_landed,
            require_airborne=require_airborne,
        )

    def hold_steady(self, *, require_airborne: bool = True) -> None:
        """Hold the pose the aircraft is at now, and wait until it is steady.

        The target is always re-taken from the current state: a hold left over
        from an earlier home move would otherwise keep defending that pose and
        drag the aircraft back down.
        """
        if self._is_landed():
            if require_airborne:
                raise ExecutionError(
                    "Cannot confirm airborne hold: vehicle is landed"
                )
            return
        self._holding.start()
        self._holding.wait()

    def takeoff(self, *, timeout_s: float | None = None) -> None:
        flight.takeoff(self, timeout_s=timeout_s)

    def land(self, *, timeout_s: float | None = None) -> bool:
        """Native land; returns the landed state measured after settling."""
        return flight.land(self, timeout_s=timeout_s)

    def return_home(self, vehicle: str | None = None) -> WorldPose:
        """Teleport to the settled spawn pose and confirm touchdown.

        ``vehicle`` selects the vehicle in a multi-vehicle session; this object is one
        vehicle, so it is ignored here and the pose it moved to is returned.
        """
        return flight.return_home(self)

    def teleport(self, pose: WorldPose) -> None:
        """Instantly move the drone without a touchdown sequence."""
        flight.teleport(self, pose)

    # -- Vehicle port ------------------------------------------------------
    @property
    def tracking_limits(self):
        return self._tracking

    def current_pose(self) -> WorldPose:
        return pose_from_kinematics(self._kinematics())

    def park(self) -> None:
        """Stop flying without waiting for a touchdown.

        Releases the position hold and cancels the last native command, which
        leaves the aircraft inert. The session calls this just before moving
        the drone out of view: the hide teleport and its altitude pin are what
        keep it in place.
        """
        self._require_started()
        self._holding.release()
        self.cancel()

    def cancel(self) -> None:
        self._cancel.set()
        self._robot.cancel_last_task()

    def hold(self) -> None:
        """Start the position hold and command one control period of it."""
        self._require_started()
        self._holding.start()
        self._holding.tick()

    def hold_tick(self) -> None:
        self._holding.tick()

    def idle_tick(self) -> None:
        """Owner-thread idle step: keep a live hold, else pace the poll.

        A live hold owns the yaw and position commands, so ticking it is the
        whole step. Without a hold the call only paces the owner loop: the
        engine advances the clock itself, and the wait costs one control
        period per poll.
        """
        if self._holding.active:
            self._holding.tick()
        else:
            self._connection.advance_for_sim_time(self._period)

    def begin_observation_hold(self) -> None:
        """Keep the existing fixed reference across consecutive captures."""
        if not self._holding.active:
            self.hold()
        else:
            self._holding.tick()

    def wait_hold(self, *, budget=None, checkpoint=None) -> None:
        self._holding.wait(budget=budget, checkpoint=checkpoint)

    def motion_capability(self) -> MotionCapability:
        """The quadrotor translates freely in the world frame."""
        return self._capability

    def collision_state(self) -> CollisionState:
        """Consume the latched non-ground contact, if any."""
        event = self._collisions.consume()
        if event is not None:
            self._last_collision = event
            return CollisionState(
                True, CollisionContact(event.object_name, time.monotonic())
            )
        return CollisionState(False)

    @property
    def last_collision(self) -> CollisionEvent | None:
        return self._last_collision

    @property
    def landed(self) -> bool:
        """Controller flag or measured touchdown at the captured altitude."""
        return self._is_landed()

    # -- streaming port ----------------------------------------------------
    def stream_state(self) -> TimedVehicleState:
        return state_from_kinematics(self._kinematics(), time.monotonic())

    def stream_command(self, command: MotionCommand, duration_s: float) -> None:
        require_supported(command, self._capability)
        self._holding.release()
        self.hold_velocity(
            (command.vx, command.vy, command.vz), command.yaw_rate_deg_s, duration_s
        )

    def hold_velocity(
        self,
        velocity_world: tuple[float, float, float],
        yaw_rate_deg_s: float,
        duration_s: float,
    ) -> None:
        self._require_started()
        self._commands.move_by_velocity(
            velocity_world, yaw_rate_deg_s, duration_s
        )

    # -- observation -------------------------------------------------------
    def capture(self, *, budget=None, timeout_s: float = 2.0) -> Observation:
        """Take a stopped observation once the vehicle has settled."""
        return self._observations.capture_settled(
            budget=budget, timeout_s=timeout_s
        )

    def capture_moving(self, *, budget=None) -> Observation:
        return self._observations.capture_moving(budget=budget)

    def _settle_capture(self, *, budget=None) -> None:
        """Settle hook: a live hold is the gate a stopped frame waits behind."""
        if self._holding.active:
            self._holding.wait(budget=budget)

    # -- internals ---------------------------------------------------------
    def _require_started(self) -> None:
        if not self._started:
            raise ExecutionError(
                "drone API control is not enabled; call start() first"
            )

    def _kinematics(self) -> dict:
        return self._connection.request(
            f"{self._robot.parent_topic}/GetGroundTruthKinematics"
        )

    def _landed_state(self) -> int:
        return int(self._robot.get_landed_state())

    def _is_landed(self) -> bool:
        """Controller flag or measured touchdown at the captured altitude.

        The fast-physics controller reports FLYING after a completed land, so
        the ground altitude measured before takeoff is the authoritative
        landing evidence once the airborne climb has been observed.
        """
        if self._landed_state() == int(LandedState.LANDED):
            return True
        if self._ground_z is None:
            return False
        return self.current_pose().z >= self._ground_z - 0.1

    def close(self) -> None:
        try:
            self._collisions.close()
        finally:
            try:
                self._observations.close()
            finally:
                self._bridge.close()
