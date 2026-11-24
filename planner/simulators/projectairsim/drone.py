"""ProjectAirSim Drone adapter: pose, takeoff/land, velocity flight, observations.

The drone is a ``fast-physics`` SimpleFlight quadrotor. ``position_hold`` is
the reference owner-thread hold service: every command is a ``MoveByVelocity``
for one control period. The simulator clock runs free after scene load, so a
command does not have to wait for the planner to step the world.
Native takeoff is verified by measured climb and landed state because
ProjectAirSim returns ``False`` for a takeoff that actually lifted off (the
reference Planner tolerates the same quirk). A completed land also leaves
``landed_state`` at FLYING, so touchdown is confirmed from the ground altitude
captured before takeoff.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from collections.abc import Sequence

from projectairsim.drone import YawControlMode
from projectairsim.types import LandedState

from ...config import CameraSettings, DepthSettings
from ...domain.observation import Observation
from ...domain.pose import WorldPose
from ...domain.streaming import TimedVehicleState
from ...errors import ExecutionError
from ...services.motion.synchronized import SMOOTH_PROFILE
from ...services.position_hold import PositionHold
from ...services.segment_motion import MotionLimits
from .async_bridge import AsyncBridge
from .collision import CollisionEvent, CollisionMonitor
from .connection import ProjectAirSimConnection
from .coordinates import (
    attitude_deg_from_kinematics,
    pose_from_kinematics,
    velocity_from_kinematics,
    yaw_rate_deg_from_kinematics,
)
from .motion import SteppedMotionTracker, tracking_limits
from .observation import ProjectAirSimObservationSource

logger = logging.getLogger(__name__)

# ``SetObjectPose`` refuses ("Unable to move object ... not movable") when the
# actor is already at the requested pose. That happens whenever a vehicle is
# homed right after being restored from its hidden parking spot, because both
# moves target the same pose. The fast-physics state is authoritative and the
# rendered actor syncs on the next clock advance, so a teleport to where the
# drone already is can skip the actor move instead of failing.
TELEPORT_TOLERANCE_M = 0.01
TELEPORT_TOLERANCE_DEG = 0.5

#: How close to a path point counts as having reached it. ``MoveToPosition``
#: answers with an ack, not with the arrival, so every point is measured here.
#: It must stay above the engine's own ``distance accuracy`` (0.5 m): when the
#: next temporary target falls inside that distance the engine commands zero
#: velocity and the aircraft stops (ProjectAirSim issue 241), so a smaller
#: tolerance can neither move the aircraft nor report the arrival.
PATH_ARRIVAL_TOLERANCE_M = 1.0

#: How long one point may take before the point is given up and the next one is
#: tried. The engine stops a position command about 0.5 m short of its target
#: (``GetDistanceAccuracy``), so a point the aircraft cannot close must not hold
#: the whole path for the caller's full budget.
PATH_POINT_WAIT_S = 20.0

#: How often the pose is read while waiting for a point to be reached, for a
#: clock the engine runs itself. The whole client shares one socket with the
#: native command channel: polling per control period -- each read a request --
#: starved the engine and the aircraft never moved at all.
ARRIVAL_POLL_S = 0.2


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
        from ...domain.motion import ContinuousTrackingSettings

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
        self._hold_tracking = tracking or ContinuousTrackingSettings()
        self._period = 1.0 / self._tracking.control_hz
        self._cancel = threading.Event()
        self._started = False
        self._bridge = AsyncBridge()
        self._observations = ProjectAirSimObservationSource(
            connection,
            robot,
            camera=camera,
            depth=depth,
            camera_id=camera_id,
            frame_sink=frame_sink,
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
        self._tracker = SteppedMotionTracker(
            self._apply_reference,
            self.current_pose,
            self._tracking,
            self._motion_timeout_s,
            clock=clock,
            cancelled=self._cancel.is_set,
            tracking=self._hold_tracking,
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
        self._require_started()
        if takeoff_if_landed:
            state = self._wait_ready()
            if self._home_pose is None:
                self._home_pose = state.pose
            landed = self._is_landed()
            if landed:
                initial_z = state.pose.z
                self._ground_z = state.pose.z
                self.cancel()
                takeoff_error = None
                try:
                    self.takeoff()
                except ExecutionError as exc:
                    takeoff_error = exc
                pose = self.current_pose()
                climb = initial_z - pose.z
                still_landed = self._is_landed()
                logger.info(
                    "Takeoff result: landed=%s climb=%.3fm initial_z=%.3f "
                    "actual_z=%.3f native_confirmed=%s",
                    still_landed,
                    climb,
                    initial_z,
                    pose.z,
                    takeoff_error is None,
                )
                if still_landed:
                    raise ExecutionError(
                        "Takeoff returned but the vehicle is still landed"
                    )
                if climb <= self._tracking.position_tolerance_m:
                    raise ExecutionError(
                        "Takeoff did not produce a measured climb from the "
                        "ground position"
                    ) from takeoff_error
                if takeoff_error is not None:
                    logger.warning(
                        "Native takeoff was not confirmed; measured climb "
                        "exists. Position hold must confirm readiness."
                    )
            self.hold(require_airborne=True)
            if landed:
                self._holding.climb_by(self._takeoff_climb_m)
        else:
            self.cancel()
            self.hold(require_airborne=require_airborne)

    def hold(self, *, require_airborne: bool = True) -> None:
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
        self._require_started()
        timeout = (
            self._takeoff_timeout_s if timeout_s is None else float(timeout_s)
        )
        self.cancel()
        # From here the native takeoff owns the aircraft.
        self._holding.release()
        try:
            result = self._bridge.run(
                self._timed_task(
                    self._robot.takeoff_async(timeout_sec=timeout),
                    step_s=0.1,
                    deadline_s=timeout,
                ),
                timeout_s=timeout + 10.0,
            )
        except ExecutionError:
            raise
        except Exception as exc:
            raise ExecutionError("ProjectAirSim takeoff did not complete") from exc
        if result is not True:
            raise ExecutionError("ProjectAirSim takeoff did not complete")

    def land(self, *, timeout_s: float | None = None) -> bool:
        """Native land; returns the landed state measured after settling."""
        self._require_started()
        timeout = self._land_timeout_s if timeout_s is None else float(timeout_s)
        self.cancel()
        self._holding.release()
        state = self.stream_state()
        logger.info(
            "Landing: pose=(%.2f,%.2f,%.2f) speed=%.2f landed=%s",
            state.pose.x,
            state.pose.y,
            state.pose.z,
            math.hypot(*state.velocity_world),
            self._is_landed(),
        )
        try:
            result = self._bridge.run(
                self._timed_task(
                    self._robot.land_async(timeout_sec=timeout),
                    step_s=0.1,
                    deadline_s=timeout,
                ),
                timeout_s=max(120.0, timeout * 4.0),
            )
        except ExecutionError:
            raise
        except Exception as exc:
            # 消息里显式带异常对象，便于按类型检索日志。
            logger.exception(
                "Land bridge failure: %r bridge_alive=%s",
                exc,  # noqa: TRY401
                self._bridge.alive,
            )
            raise ExecutionError("ProjectAirSim land did not complete") from exc
        if result is not True:
            logger.warning(
                "ProjectAirSim land returned %r bridge_alive=%s",
                result,
                self._bridge.alive,
            )
            raise ExecutionError("ProjectAirSim land did not complete")
        # The native task completes at touchdown order, but ``landed_state``
        # only flips after the fast-physics contact settles. Keep letting the
        # clock advance so the measured state is meaningful.
        pose = self.current_pose()
        landed = self._is_landed()
        settled_s = 0.0
        while not landed and settled_s < self._land_settle_timeout_s:
            self._connection.advance_for_sim_time(0.5)
            settled_s += 0.5
            pose = self.current_pose()
            landed = self._is_landed()
        logger.info(
            "Land result: landed=%s pose=(%.2f,%.2f,%.2f) settle=%.1fs",
            landed,
            pose.x,
            pose.y,
            pose.z,
            settled_s,
        )
        return landed

    def return_home(self) -> WorldPose:
        """Teleport to the settled spawn pose and confirm touchdown.

        The reference rover home semantics apply to the drone as an instant
        teleport (``SetPose`` with ``reset_kinematics``), not ``go_home``: the
        pose is applied at once and touchdown is confirmed rather than flown to.
        If the pose does not converge, the teleport is repeated once. Touchdown
        is confirmed from ``_is_landed`` because the native ``landed_state`` can
        stay FLYING after a real landing (see the module docstring). The move
        ends with the position hold armed at the spawn, so the aircraft is
        commanded here rather than by whatever goal the engine was last given.
        """
        self._require_started()
        self._holding.release()
        self.cancel()
        if self._home_pose is None:
            state = self._wait_ready()
            self._home_pose = state.pose
            if self._ground_z is None:
                self._ground_z = state.pose.z
        home = self._home_pose
        if self._ground_z is None:
            # The spawn pose is the ground reference for touchdown checks.
            self._ground_z = home.z
        elif self._ground_z > home.z:
            # The authored spawn can float above the surface the vehicle really
            # rests on: this map's spawn is 1.4 m up, so returning to the
            # authored z left the drone hovering, ``_is_landed`` never became
            # true, and the settle wait below spent its whole timeout while the
            # engine's stale goal lifted the drone away. The measured ground is
            # the height a returned drone has to reach.
            home = WorldPose(home.x, home.y, self._ground_z, home.yaw_deg)
        last_error: Exception | None = None
        for attempt in range(2):
            try:
                self._set_pose(home)
                for _ in range(2):
                    self._connection.advance_for_sim_time(self._period)
                pose = self.current_pose()
                distance = math.dist(
                    (pose.x, pose.y, pose.z), (home.x, home.y, home.z)
                )
                yaw_error = abs(
                    (pose.yaw_deg - home.yaw_deg + 180.0) % 360.0 - 180.0
                )
                if distance > 0.25 or yaw_error > 3.0:
                    raise ExecutionError(
                        "drone home pose did not converge: "
                        f"distance={distance:.2f}m yaw_error={yaw_error:.1f}deg"
                    )
                settled = 0.0
                while not self._is_landed() and settled < self._land_settle_timeout_s:
                    self._connection.advance_for_sim_time(0.5)
                    settled += 0.5
                pose = self.current_pose()
                self._collisions.clear()
                if self._ground_z is None:
                    self._ground_z = pose.z
                # The engine keeps flying the goal it was last given, and a
                # teleport does not change that goal. The hold commands a
                # velocity every control period, so arming it here takes that
                # goal away from the previous task's climb; ``idle_tick`` keeps
                # ticking it afterwards.
                self._holding.start(target=home)
                logger.info(
                    "Drone returned home: pose=(%.2f,%.2f,%.2f) yaw=%.1f "
                    "landed=%s ground_z=%.2f settle=%.1fs",
                    pose.x,
                    pose.y,
                    pose.z,
                    pose.yaw_deg,
                    self._is_landed(),
                    self._ground_z,
                    settled,
                )
                return pose
            except Exception as error:
                # A pose that did not arrive means the teleport did not take.
                # Landing as a recovery would aim at the ground under wherever
                # the drone is, not at its spawn.
                last_error = error
                logger.warning(
                    "Drone home attempt %d failed: %s", attempt + 1, error
                )
        raise ExecutionError(
            f"drone could not return home: {last_error}"
        ) from last_error

    def teleport(self, pose: WorldPose) -> None:
        """Instantly move the drone without a touchdown sequence.

        Used to move an inactive drone out of the streaming view (and back):
        ``SetPose`` with ``reset_kinematics`` makes the idle teleport
        authoritative while the active vehicle keeps stepping the world.
        """
        self._require_started()
        self._holding.release()
        self.cancel()
        self._set_pose(pose)
        self._collisions.clear()

    def _set_pose(self, pose: WorldPose) -> None:
        quaternion = (
            math.cos(math.radians(pose.yaw_deg) / 2),
            math.sin(math.radians(pose.yaw_deg) / 2),
        )
        transform = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": pose.x, "y": pose.y, "z": pose.z},
            "rotation": {
                "w": quaternion[0],
                "x": 0.0,
                "y": 0.0,
                "z": quaternion[1],
            },
        }
        current = self.current_pose()
        self._connection.request(
            f"{self._robot.parent_topic}/SetPose",
            {
                "pose": transform,
                "reset_kinematics": True,
            },
        )
        # SetPose moves the fast-physics state but not the rendered actor, so
        # a returned/hidden drone would stay visible where it was. Move the
        # actor explicitly with the same target pose -- unless it is already
        # there, which the server refuses (see TELEPORT_TOLERANCE_M).
        distance = math.dist(
            (current.x, current.y, current.z), (pose.x, pose.y, pose.z)
        )
        yaw_error = abs(
            (current.yaw_deg - pose.yaw_deg + 180.0) % 360.0 - 180.0
        )
        if (
            distance > TELEPORT_TOLERANCE_M
            or yaw_error > TELEPORT_TOLERANCE_DEG
        ):
            self._connection.request(
                f"{self._robot.world_parent_topic}/SetObjectPose",
                {
                    "object_name": self._robot.name,
                    "pose": transform,
                    "teleport": True,
                },
            )

    # -- Vehicle port ------------------------------------------------------
    @property
    def tracking_limits(self):
        return self._tracking

    def current_pose(self) -> WorldPose:
        return pose_from_kinematics(self._kinematics())

    def execute_segment(
        self, segment, *, budget=None, pose_checkpoint=None
    ) -> None:
        self._require_started()
        self._holding.release()
        if segment.profile != SMOOTH_PROFILE:
            raise ExecutionError(
                f"unsupported motion profile: {segment.profile}"
            )
        self._cancel.clear()
        self._tracker.execute(
            segment, budget=budget, pose_checkpoint=pose_checkpoint
        )
        self._holding.start(segment.end)
        self._holding.wait(budget=budget)

    def follow_path(
        self, points: Sequence[WorldPose], speed_mps: float, *, timeout_s: float
    ) -> None:
        """Fly one world-frame path and wait until the aircraft has reached its end.

        The port hands over the whole path; the adapter drives it point by point
        through ``MoveToPosition``. This scene rejects ``MoveOnPath`` outright --
        the server answers "Requested ServiceMethod .../MoveOnPath is not
        supported by Request-Response Service" while the native call still returns
        ``True``, which is why a path handed to it was accepted and never flown --
        and ``eval/evalcore/sim.py`` records the same conclusion and flies
        waypoints the same way.

        Each point is one engine command, given the caller's remaining deadline as
        its ``timeout_sec``. The engine answers when it takes a command, not when
        the aircraft arrives, so arrival is measured here; the package's
        ``move_to_position_async`` says so itself ("Control returns back to the
        caller immediately"). Whichever way the path ends -- flown, timed out or
        failed -- the hold starts again before this method returns or raises, so
        the aircraft does not coast into the next round.
        """
        self._require_started()
        if not points:
            raise ExecutionError("path following needs at least one point")
        self._holding.release()
        self._cancel.clear()
        issued = time.monotonic()
        deadline = issued + float(timeout_s)
        arrived = False
        try:
            # ``MoveOnPath`` is not a service this scene registers -- the server
            # answers "not supported by Request-Response Service" -- and where it
            # does register it, the aircraft does not move (ProjectAirSim issue
            # 81). ``MoveToPosition`` does move it, and `eval/evalcore/sim.py:265`
            # records the same conclusion for this build, so the path is driven
            # point by point. The port still hands over the whole path: driving it
            # to the last point is the adapter's job.
            for index, target in enumerate(points, start=1):
                if self._is_reached(target):
                    logger.info(
                        "[PathFollow] point %d already within %.1fm; skipping",
                        index, PATH_ARRIVAL_TOLERANCE_M,
                    )
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.cancel()
                    raise ExecutionError(
                        "ProjectAirSim path following did not arrive: "
                        f"point {index} was not reached in {timeout_s:g}s"
                    )
                point_deadline = min(deadline, time.monotonic() + PATH_POINT_WAIT_S)
                self._bridge.run(
                    self._move_exchange(target, float(speed_mps), remaining),
                    timeout_s=remaining,
                )
                # A point the aircraft does not close is skipped rather than
                # failing the path: the engine's own arrival check is looser than
                # ours can be, and the next round replans from the measured pose.
                self._await_arrival(target, point_deadline, index)
            arrived = True
        except ExecutionError:
            raise
        except TimeoutError as exc:
            # Nothing else stops the native path, so the vehicle would keep
            # flying after the caller gave up.
            raise ExecutionError(
                f"ProjectAirSim path following did not complete in {timeout_s:g}s"
            ) from exc
        except Exception as exc:
            logger.warning(
                "[PathFollow] request failed after %.2fs: %r",
                time.monotonic() - issued, exc,
            )
            # The original reason has to survive: ``ProjectAirSim path following
            # failed`` alone says nothing about a TypeError, a dropped socket or a
            # package quirk.
            raise ExecutionError(
                f"ProjectAirSim path following failed: {type(exc).__name__}: {exc}"
            ) from exc
        finally:
            # A flown path ends at its last point; a refused or abandoned one ends
            # wherever the aircraft is, so the measured pose is the reference.
            self._restore_hold(points[-1] if arrived else None)

    def _is_reached(self, target: WorldPose) -> bool:
        """Return whether the aircraft is already close enough to ``target``."""
        pose = self.current_pose()
        return math.dist(
            (pose.x, pose.y, pose.z), (target.x, target.y, target.z)
        ) <= PATH_ARRIVAL_TOLERANCE_M

    def _await_arrival(
        self, target: WorldPose, deadline: float, index: int
    ) -> None:
        """Wait for the pose to reach one path point, within its own deadline.

        ``MoveToPosition`` answers when the engine takes the command, not when the
        aircraft gets there, so the distance is measured here. The clock decides
        the pace: a free-running engine needs no push and one pose read every
        ``ARRIVAL_POLL_S`` leaves the socket to the native task, while a stepped
        world only moves when a control period asks it to and keeps the owner
        loop's step.

        A point that is still out of reach at its deadline is reported and left
        behind. The engine gives up on a position about 0.5 m short of its target,
        so waiting longer cannot close it, and the caller replans from the
        measured pose on the next round.
        """
        free_running = bool(self._connection.free_running)
        steps = 0
        while True:
            steps += 1
            pose = self.current_pose()
            distance = math.dist(
                (pose.x, pose.y, pose.z), (target.x, target.y, target.z)
            )
            if distance <= PATH_ARRIVAL_TOLERANCE_M:
                logger.info(
                    "[PathFollow] point %d reached: pose=(%.2f,%.2f,%.2f) "
                    "distance=%.2fm pose_reads=%d",
                    index, pose.x, pose.y, pose.z, distance, steps,
                )
                return
            if time.monotonic() >= deadline:
                logger.info(
                    "[PathFollow] point %d given up at %.2fm: the engine stops "
                    "about 0.5m short of a position, the next point continues",
                    index, distance,
                )
                return
            if free_running:
                self._sleep(ARRIVAL_POLL_S)
            else:
                self._connection.advance_for_sim_time(self._period)

    def _restore_hold(self, target: WorldPose | None = None) -> None:
        """Return to a controlled hover after the path, by any route.

        Without a ``target`` the hold takes the measured pose, which is what a
        refused or abandoned path leaves behind. Starting the hold also cancels
        the native task before it commands the first period, so this is what stops
        a path the engine refused or one the caller gave up on. It never raises:
        the caller is already reporting why the path ended, and stopping is exactly
        what a broken transport makes fail.
        """
        try:
            self._require_started()
            self._holding.start(target)
            self._holding.tick()
        except Exception as error:
            logger.warning("[PathFollow] restoring the hold failed: %r", error)

    async def _move_exchange(self, target: WorldPose, speed_mps: float, timeout_s: float):
        """Ask ``MoveToPosition`` for one path point and return the engine's ack."""
        # ``ForwardOnly`` is the baseline's yaw intent, but on this build the
        # motion services reject it (ProjectAirSim issue 81: the server raises a
        # std::exception) and the aircraft does not move. ``MaxDegreeOfFreedom``
        # with a zero yaw rate is the combination that flies here.
        yaw_control_mode = YawControlMode.MaxDegreeOfFreedom
        yaw_is_rate = True
        yaw = 0.0
        logger.info(
            "[PathFollow] point target=(%.2f,%.2f,%.2f) velocity=%.2fm/s "
            "timeout_sec=%.1fs yaw_control=%s yaw_is_rate=%s yaw=%.1f",
            target.x, target.y, target.z, speed_mps, timeout_s,
            getattr(yaw_control_mode, "name", yaw_control_mode), yaw_is_rate, yaw,
        )
        task = await self._robot.move_to_position_async(
            float(target.x),
            float(target.y),
            float(target.z),
            velocity=speed_mps,
            timeout_sec=timeout_s,
            yaw_control_mode=yaw_control_mode,
            yaw_is_rate=yaw_is_rate,
            yaw=yaw,
        )
        return await task

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

    def hover(self) -> None:
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
            self.hover()
        else:
            self._holding.tick()

    def wait_hold(self, *, budget=None, checkpoint=None) -> None:
        self._holding.wait(budget=budget, checkpoint=checkpoint)

    def collision_state(self) -> bool:
        """Consume the latched non-ground contact, if any."""
        event = self._collisions.consume()
        if event is not None:
            self._last_collision = event
            return True
        return False

    @property
    def last_collision(self) -> CollisionEvent | None:
        return self._last_collision

    @property
    def landed(self) -> bool:
        """Controller flag or measured touchdown at the captured altitude."""
        return self._is_landed()

    # -- streaming port ----------------------------------------------------
    def stream_state(self) -> TimedVehicleState:
        kinematics = self._kinematics()
        pose = pose_from_kinematics(kinematics)
        timestamp_ns = int(kinematics["time_stamp"])
        velocity = velocity_from_kinematics(kinematics)
        roll, pitch, _ = attitude_deg_from_kinematics(kinematics)
        return TimedVehicleState(
            pose,
            timestamp_ns,
            time.monotonic(),
            velocity,
            pitch_deg=pitch,
            roll_deg=roll,
            yaw_rate_deg_s=yaw_rate_deg_from_kinematics(kinematics),
        )

    def stream_velocity(
        self,
        velocity_world: tuple[float, float, float],
        yaw_rate_deg_s: float,
        duration_s: float,
    ) -> None:
        self._holding.release()
        self.hold_velocity(velocity_world, yaw_rate_deg_s, duration_s)

    def hold_velocity(
        self,
        velocity_world: tuple[float, float, float],
        yaw_rate_deg_s: float,
        duration_s: float,
    ) -> None:
        self._require_started()
        if (
            len(velocity_world) != 3
            or not all(
                math.isfinite(value)
                for value in (*velocity_world, yaw_rate_deg_s, duration_s)
            )
            or duration_s <= 0
        ):
            raise ExecutionError(
                "non-finite streaming velocity or invalid duration"
            )
        try:
            result = self._bridge.run(
                self._velocity_exchange(
                    velocity_world, yaw_rate_deg_s, duration_s
                ),
                timeout_s=max(10.0, duration_s * 20.0),
            )
        except ExecutionError:
            raise
        except Exception as exc:
            raise ExecutionError(
                "ProjectAirSim velocity command failed"
            ) from exc
        if result is not True:
            raise ExecutionError("ProjectAirSim velocity command was interrupted")

    async def _velocity_exchange(
        self,
        velocity_world: tuple[float, float, float],
        yaw_rate_deg_s: float,
        duration_s: float,
    ):
        task = await self._robot.move_by_velocity_async(
            float(velocity_world[0]),
            float(velocity_world[1]),
            float(velocity_world[2]),
            duration=float(duration_s),
            yaw_control_mode=YawControlMode.MaxDegreeOfFreedom,
            yaw_is_rate=True,
            yaw=math.radians(float(yaw_rate_deg_s)),
        )
        self._connection.advance_for_sim_time(duration_s)
        return await task

    # -- observation -------------------------------------------------------
    def capture(self, *, budget=None, timeout_s: float = 2.0) -> Observation:
        """Take a stopped observation, after the vehicle has settled.

        ``stopped`` has to be true at image time, not at request time. A pause
        that fires mid-flight leaves several metres of coasting: the frame
        source would hand out an image whose pose no longer matches the vehicle
        a second later, and the caller discards it as a changed anchor. The
        UE4 adapter gets this guarantee from its observation gate; here the
        position hold is the gate.
        """
        if self._holding.active:
            self._holding.wait(budget=budget)
        return self._observations.capture(budget=budget, timeout_s=timeout_s)

    def capture_moving(self, *, budget=None) -> Observation:
        return self._observations.capture_moving(budget=budget)

    # -- internals ---------------------------------------------------------
    def _apply_reference(
        self,
        velocity_world: tuple[float, float, float],
        yaw_rate_deg_s: float,
        measured_velocity_world: tuple[float, float, float],
        yaw_deg: float,
        period_s: float,
    ) -> None:
        self.hold_velocity(velocity_world, yaw_rate_deg_s, period_s)

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

    async def _timed_task(
        self, command, *, step_s: float, deadline_s: float
    ):
        task = await command
        advanced = 0.0
        reported = 0.0
        while not task.done():
            if advanced >= deadline_s:
                self._robot.cancel_last_task()
                raise ExecutionError(
                    "native command exceeded its simulator-time budget"
                )
            self._connection.advance_for_sim_time(step_s)
            advanced += step_s
            if advanced - reported >= 5.0:
                logger.info(
                    "Native command still running: sim_advanced=%.1fs",
                    advanced,
                )
                reported = advanced
            await asyncio.sleep(0.005)
        return task.result()

    def _wait_ready(self) -> TimedVehicleState:
        began = self._clock()
        stable_since_ns = None
        previous_ns, previous_landed = None, None
        while self._clock() - began < self._motion_timeout_s:
            state = self.stream_state()
            landed = self._landed_state()
            quiet = (
                landed != int(LandedState.LANDED)
                or math.hypot(*state.velocity_world)
                <= self._tracking.stopped_speed_mps
            )
            advancing = (
                previous_ns is not None
                and state.timestamp_ns > previous_ns
                and landed == previous_landed
            )
            if advancing and quiet:
                # Simulator time, not wall time, defines the stable window: the
                # engine's clock may run faster or slower than real time,
                # exactly like PositionHold's observable window.
                if stable_since_ns is None:
                    stable_since_ns = state.timestamp_ns
                if (
                    state.timestamp_ns - stable_since_ns
                    >= self._readiness_stable_s * 1e9
                ):
                    return state
            else:
                stable_since_ns = None
            previous_ns, previous_landed = state.timestamp_ns, landed
            self._connection.advance_for_sim_time(
                self._readiness_poll_interval_s
            )
        raise ExecutionError(
            "ProjectAirSim state did not become ready before the startup timeout"
        )

    def close(self) -> None:
        try:
            self._collisions.close()
        finally:
            try:
                self._observations.close()
            finally:
                self._bridge.close()