"""ProjectAirSim Drone adapter: pose, takeoff/land, velocity flight, observations.

The drone is a ``fast-physics`` SimpleFlight quadrotor. ``position_hold`` is
the reference owner-thread hold service: every command is a
``MoveByVelocity`` followed by exactly one control-period clock advance, so the
paused steppable world stays in lockstep with the shared execution stack.
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
        self._stream_pose: WorldPose | None = None
        self._stream_ns = 0
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
        else:
            self.cancel()
            self.hold(require_airborne=require_airborne)

    def hold(self, *, require_airborne: bool = True) -> None:
        if self._is_landed():
            if require_airborne:
                raise ExecutionError(
                    "Cannot confirm airborne hold: vehicle is landed"
                )
            return
        if not self._holding.active:
            self._holding.start()
        self._holding.wait()

    def takeoff(self, *, timeout_s: float | None = None) -> None:
        self._require_started()
        timeout = (
            self._takeoff_timeout_s if timeout_s is None else float(timeout_s)
        )
        self.cancel()
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
            logger.exception(
                "Land bridge failure: %r bridge_alive=%s",
                exc,
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
        # only flips after the fast-physics contact settles. Keep stepping the
        # paused world so the measured state is meaningful.
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
        spawn altitude is on the ground, so the vehicle ends parked. If the
        pose does not converge within tolerance, land and retry once. Touchdown
        is confirmed from ``_is_landed`` because the native ``landed_state``
        can stay FLYING after a real landing (see the module docstring).
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
                self._stream_pose, self._stream_ns = None, 0
                self._collisions.clear()
                if self._ground_z is None:
                    self._ground_z = pose.z
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
                last_error = error
                logger.warning(
                    "Drone home attempt %d failed: %s", attempt + 1, error
                )
                if attempt == 0:
                    try:
                        self.land()
                    except Exception as land_error:
                        logger.warning(
                            "Drone home recovery land failed: %s", land_error
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
        self._stream_pose, self._stream_ns = None, 0
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
        """Owner-thread idle step: keep a live hold, else advance the clock.

        The paused world only moves when the owner advances it, so an idle
        viewer still needs one control period per poll. A landed vehicle has
        no hold to tick and the bare clock advance keeps the scene rendering.
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
        previous, previous_ns = self._stream_pose, self._stream_ns
        velocity = (0.0, 0.0, 0.0)
        if previous is not None and timestamp_ns > previous_ns:
            dt = (timestamp_ns - previous_ns) / 1e9
            velocity = (
                (pose.x - previous.x) / dt,
                (pose.y - previous.y) / dt,
                (pose.z - previous.z) / dt,
            )
        self._stream_pose, self._stream_ns = pose, timestamp_ns
        roll, pitch, _ = attitude_deg_from_kinematics(kinematics)
        return TimedVehicleState(
            pose,
            timestamp_ns,
            time.monotonic(),
            velocity,
            pitch_deg=pitch,
            roll_deg=roll,
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
                # The stepped clock, not wall time, defines the stable window:
                # advancing the paused world may be faster or slower than real
                # time, exactly like PositionHold's observable window.
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