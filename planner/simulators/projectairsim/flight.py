"""ProjectAirSim drone flight sequences: readiness, takeoff, land, home.

These are the multi-step native sequences the drone class exposes. They read
the measured state, drive the command downlink in ``commands`` and keep the
position hold consistent, but they own no connection of their own: each
function takes the drone adapter and works on its state. A native takeoff is
verified by measured climb and landed state because ProjectAirSim returns
``False`` for a takeoff that actually lifted off (the reference Planner
tolerates the same quirk). A completed land also leaves ``landed_state`` at
FLYING, so touchdown is confirmed from the ground altitude captured before
takeoff.
"""

from __future__ import annotations

import logging
import math

from planner.domain import WorldPose
from planner.domain import TimedVehicleState
from projectairsim.types import LandedState

from ...errors import ExecutionError

logger = logging.getLogger(__name__)


def prepare(
    drone, *, takeoff_if_landed: bool = True, require_airborne: bool = False
) -> None:
    """Reference readiness sequence: tolerant native takeoff, then hold."""
    drone._require_started()
    if takeoff_if_landed:
        state = wait_ready(drone)
        if drone._home_pose is None:
            drone._home_pose = state.pose
        landed = drone._is_landed()
        if landed:
            initial_z = state.pose.z
            drone._ground_z = state.pose.z
            drone.cancel()
            takeoff_error = None
            try:
                drone.takeoff()
            except ExecutionError as exc:
                takeoff_error = exc
            pose = drone.current_pose()
            climb = initial_z - pose.z
            still_landed = drone._is_landed()
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
            if climb <= drone._tracking.position_tolerance_m:
                raise ExecutionError(
                    "Takeoff did not produce a measured climb from the "
                    "ground position"
                ) from takeoff_error
            if takeoff_error is not None:
                logger.warning(
                    "Native takeoff was not confirmed; measured climb "
                    "exists. Position hold must confirm readiness."
                )
        drone.hold(require_airborne=True)
        if landed:
            drone._holding.climb_by(drone._takeoff_climb_m)
    else:
        drone.cancel()
        drone.hold(require_airborne=require_airborne)


def takeoff(drone, *, timeout_s: float | None = None) -> None:
    drone._require_started()
    timeout = (
        drone._takeoff_timeout_s if timeout_s is None else float(timeout_s)
    )
    drone.cancel()
    # From here the native takeoff owns the aircraft.
    drone._holding.release()
    try:
        result = drone._commands.run_native(
            drone._robot.takeoff_async(timeout_sec=timeout),
            step_s=0.1,
            deadline_s=timeout,
            timeout_s=timeout + 10.0,
        )
    except ExecutionError:
        raise
    except Exception as exc:
        raise ExecutionError("ProjectAirSim takeoff did not complete") from exc
    if result is not True:
        raise ExecutionError("ProjectAirSim takeoff did not complete")


def land(drone, *, timeout_s: float | None = None) -> bool:
    """Native land; returns the landed state measured after settling."""
    drone._require_started()
    timeout = drone._land_timeout_s if timeout_s is None else float(timeout_s)
    drone.cancel()
    drone._holding.release()
    state = drone.stream_state()
    logger.info(
        "Landing: pose=(%.2f,%.2f,%.2f) speed=%.2f landed=%s",
        state.pose.x,
        state.pose.y,
        state.pose.z,
        math.hypot(*state.velocity_world),
        drone._is_landed(),
    )
    try:
        result = drone._commands.run_native(
            drone._robot.land_async(timeout_sec=timeout),
            step_s=0.1,
            deadline_s=timeout,
            timeout_s=max(120.0, timeout * 4.0),
        )
    except ExecutionError:
        raise
    except Exception as exc:
        # 消息里显式带异常对象，便于按类型检索日志。
        logger.exception(
            "Land bridge failure: %r bridge_alive=%s",
            exc,  # noqa: TRY401
            drone._bridge.alive,
        )
        raise ExecutionError("ProjectAirSim land did not complete") from exc
    if result is not True:
        logger.warning(
            "ProjectAirSim land returned %r bridge_alive=%s",
            result,
            drone._bridge.alive,
        )
        raise ExecutionError("ProjectAirSim land did not complete")
    # The native task completes at touchdown order, but ``landed_state``
    # only flips after the fast-physics contact settles. Keep letting the
    # clock advance so the measured state is meaningful.
    pose = drone.current_pose()
    landed = drone._is_landed()
    settled_s = 0.0
    while not landed and settled_s < drone._land_settle_timeout_s:
        drone._connection.advance_for_sim_time(0.5)
        settled_s += 0.5
        pose = drone.current_pose()
        landed = drone._is_landed()
    logger.info(
        "Land result: landed=%s pose=(%.2f,%.2f,%.2f) settle=%.1fs",
        landed,
        pose.x,
        pose.y,
        pose.z,
        settled_s,
    )
    return landed


def return_home(drone) -> WorldPose:
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
    drone._require_started()
    drone._holding.release()
    drone.cancel()
    if drone._home_pose is None:
        state = wait_ready(drone)
        drone._home_pose = state.pose
        if drone._ground_z is None:
            drone._ground_z = state.pose.z
    home = drone._home_pose
    if drone._ground_z is None:
        # The spawn pose is the ground reference for touchdown checks.
        drone._ground_z = home.z
    elif drone._ground_z > home.z:
        # The authored spawn can float above the surface the vehicle really
        # rests on: this map's spawn is 1.4 m up, so returning to the
        # authored z left the drone hovering, ``_is_landed`` never became
        # true, and the settle wait below spent its whole timeout while the
        # engine's stale goal lifted the drone away. The measured ground is
        # the height a returned drone has to reach.
        home = WorldPose(home.x, home.y, drone._ground_z, home.yaw_deg)
    last_error: Exception | None = None
    for attempt in range(2):
        try:
            drone._commands.set_pose(home, current=drone.current_pose())
            for _ in range(2):
                drone._connection.advance_for_sim_time(drone._period)
            pose = drone.current_pose()
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
            while (
                not drone._is_landed()
                and settled < drone._land_settle_timeout_s
            ):
                drone._connection.advance_for_sim_time(0.5)
                settled += 0.5
            pose = drone.current_pose()
            drone._collisions.clear()
            if drone._ground_z is None:
                drone._ground_z = pose.z
            # The engine keeps flying the goal it was last given, and a
            # teleport does not change that goal. The hold commands a
            # velocity every control period, so arming it here takes that
            # goal away from the previous task's climb; ``idle_tick`` keeps
            # ticking it afterwards.
            drone._holding.start(target=home)
            logger.info(
                "Drone returned home: pose=(%.2f,%.2f,%.2f) yaw=%.1f "
                "landed=%s ground_z=%.2f settle=%.1fs",
                pose.x,
                pose.y,
                pose.z,
                pose.yaw_deg,
                drone._is_landed(),
                drone._ground_z,
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


def teleport(drone, pose: WorldPose) -> None:
    """Instantly move the drone without a touchdown sequence.

    Used to move an inactive drone out of the streaming view (and back):
    ``SetPose`` with ``reset_kinematics`` makes the idle teleport
    authoritative while the active vehicle keeps stepping the world.
    """
    drone._require_started()
    drone._holding.release()
    drone.cancel()
    drone._commands.set_pose(pose, current=drone.current_pose())
    drone._collisions.clear()


def wait_ready(drone) -> TimedVehicleState:
    began = drone._clock()
    stable_since_ns = None
    previous_ns, previous_landed = None, None
    while drone._clock() - began < drone._motion_timeout_s:
        state = drone.stream_state()
        landed = drone._landed_state()
        quiet = (
            landed != int(LandedState.LANDED)
            or math.hypot(*state.velocity_world)
            <= drone._tracking.stopped_speed_mps
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
                >= drone._readiness_stable_s * 1e9
            ):
                return state
        else:
            stable_since_ns = None
        previous_ns, previous_landed = state.timestamp_ns, landed
        drone._connection.advance_for_sim_time(
            drone._readiness_poll_interval_s
        )
    raise ExecutionError(
        "ProjectAirSim state did not become ready before the startup timeout"
    )
