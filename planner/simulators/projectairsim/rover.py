"""ProjectAirSim Rover adapter: pose, observations, drive and contacts.

The rover is a ``fast-physics`` actor commanded through ``SetRoverControls``.
This module provides the actuator side of approach A (the reference velocity
stream converted by the calibrated drive model) plus the read side the
workflows consume: ground-truth pose in NED, synchronized camera observations,
streaming state with a finite-differenced velocity (the reported twist is a
placeholder), and latched non-ground collision contacts.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Sequence
from dataclasses import replace

from ...config import CameraSettings, DepthSettings
from ...domain.observation import Observation
from ...domain.pose import WorldPose
from ...domain.streaming import TimedVehicleState
from ...errors import ExecutionError
from ...services.segment_motion import MotionLimits
from .async_bridge import AsyncBridge
from .collision import CollisionEvent, CollisionMonitor
from .connection import ProjectAirSimConnection
from .coordinates import (
    pose_from_kinematics,
)
from .drive import (
    RoverControls,
    RoverDriveModel,
    SpeedPid,
    controls_from_reference,
)
from .motion import SteppedMotionTracker, tracking_limits
from .observation import ProjectAirSimObservationSource

# The authored spawn point sits above the rendered road surface, so the
# fast-physics car falls about a metre after the clock starts. Measured on
# Neighborhood: the drop finishes within ~1.0 s of braking and the pose is
# stable afterwards; hold a little longer so slower maps settle too.
SETTLE_DURATION_S = 2.0

#: 阿克曼车不能横移：冲过端点后横向残差收不到中心线。给“已驶过端点”判据一条
#: 车宽量级的走廊（0.6 m），让“冲过一点但仍在走廊内”直接推进下一个航点，而不是
#: 回头精确命中一个它够不到的点——后者会退化成身后的目标，触发倒车与原地卡死。
ROVER_PASSED_LATERAL_TOLERANCE_M = 0.6


def rover_tracking_limits(motion):
    """Configured envelope -> domain tracking limits, widened for the rover."""
    limits = tracking_limits(motion)
    return replace(
        limits,
        passed_lateral_tolerance_m=max(
            limits.passed_lateral_tolerance_m, ROVER_PASSED_LATERAL_TOLERANCE_M
        ),
    )

# ``SetObjectPose`` refuses ("Unable to move object ... not movable") when the
# actor is already at the requested pose. That happens whenever a vehicle is
# homed right after being restored from its hidden parking spot, because both
# moves target the same pose. The fast-physics state is authoritative and the
# rendered actor syncs on the next clock advance, so a teleport to where the
# rover already is can skip the actor move instead of failing.
TELEPORT_TOLERANCE_M = 0.01
TELEPORT_TOLERANCE_DEG = 0.5

# The car still rides its suspension when the spawn drop ends, so one snapshot
# of the resting pose lands anywhere inside that bounce: two runs on
# Neighborhood put it 0.41 m apart, and homing to the low one buries the car in
# the road. ``measure_resting_pose`` therefore samples until a trailing window
# stops moving.
RESTING_TOLERANCE_M = 0.02
RESTING_WINDOW_S = 0.5
RESTING_TIMEOUT_S = 6.0


class ProjectAirSimRover:
    def __init__(
        self,
        connection: ProjectAirSimConnection,
        robot,
        *,
        camera: CameraSettings,
        depth: DepthSettings = DepthSettings(),
        camera_id: str = "FrontCamera",
        motion: MotionLimits | None = None,
        motion_timeout_s: float = 60.0,
        drive_model: RoverDriveModel = RoverDriveModel(),
        frame_sink=None,
        tracking=None,
    ):
        from ...domain.motion import ContinuousTrackingSettings

        self._connection = connection
        self._robot = robot
        self._motion = motion or MotionLimits()
        self._tracking = rover_tracking_limits(self._motion)
        self._hold_tracking = tracking or ContinuousTrackingSettings()
        self._drive_model = drive_model
        self._pid = SpeedPid(drive_model)
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
        # The car's reported twist is a placeholder, so the streaming state
        # differencing poses: a constant zero would blind the speed PID.
        self._stream_pose: WorldPose | None = None
        self._stream_ns = 0
        self._last_collision: CollisionEvent | None = None
        # Home pose used by ``return_home``. The raw spawn pose read here sits
        # above the road, so the session replaces it with the resting pose
        # measured by ``measure_resting_pose``.
        self._home_pose = self.current_pose()
        self._tracker = SteppedMotionTracker(
            self._apply_reference,
            self.current_pose,
            self._tracking,
            motion_timeout_s,
            cancelled=self._cancel.is_set,
            tracking=self._hold_tracking,
        )

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        """Enable API control and arm; required before any drive command."""
        if not self._robot.enable_api_control():
            raise ExecutionError("rover refused API control")
        if not self._robot.arm():
            raise ExecutionError("rover refused arm")
        self._apply_controls(0.0, 0.0, 1.0)
        self._started = True

    @property
    def tracking_limits(self) -> MotionLimits:
        """Reference motion limits derived from the configured car profile."""
        return self._tracking

    # -- Vehicle port ------------------------------------------------------
    def current_pose(self) -> WorldPose:
        return pose_from_kinematics(self._kinematics())

    def execute_segment(self, segment, *, budget=None, pose_checkpoint=None) -> None:
        self._require_started()
        self._cancel.clear()
        self._pid.reset()
        self._tracker.execute(
            segment, budget=budget, pose_checkpoint=pose_checkpoint
        )

    def follow_path(
        self, points: Sequence[WorldPose], speed_mps: float, *, timeout_s: float
    ) -> None:
        """Drive one world-frame path through ``MoveOnPath`` and wait for it.

        The parameters are the installed package's
        (``projectairsim/rover.py:199-222``, ``move_on_path_async``): ``path`` is a
        list of NED points whose z the ground vehicle ignores, ``velocity`` the
        desired speed in m/s, ``lookahead`` 3 and ``adaptive_lookahead`` 1 to
        match the path parameters the 3DG-VLN baseline flies with.

        The tracking itself is the engine's, not one continuous carrot path: in
        this package ``Rover.move_on_path_async`` visits the waypoints in order
        through ``MoveToPosition`` (SimpleDrive), so ``timeout_sec`` bounds one
        segment rather than the whole batch, and the rover has no yaw mode in
        this call (``yaw_rate_max`` stays at the package default ``-1``, no
        limit). The drone's ``ForwardOnly`` pair therefore has no counterpart
        here, and the whole path is still handed over in one request.
        """
        self._require_started()
        if not points:
            raise ExecutionError("path following needs at least one point")
        self._cancel.clear()
        self._pid.reset()
        path = [[float(point.x), float(point.y), float(point.z)] for point in points]
        try:
            result = self._bridge.run(
                self._path_exchange(path, float(speed_mps), float(timeout_s)),
                timeout_s=float(timeout_s),
            )
        except ExecutionError:
            raise
        except TimeoutError as exc:
            # ``MoveOnPath`` is the engine's own drive, so cancelling it is what
            # ends the path; the brake then keeps the car where it stopped
            # instead of letting it coast past the caller's deadline.
            self._robot.cancel_last_task()
            self.hover()
            raise ExecutionError(
                f"ProjectAirSim path following did not complete in {timeout_s:g}s"
            ) from exc
        except Exception as exc:
            raise ExecutionError("ProjectAirSim path following failed") from exc
        if result is False:
            raise ExecutionError("ProjectAirSim refused the path")

    async def _path_exchange(
        self, path: list[list[float]], speed_mps: float, timeout_s: float
    ):
        task = await self._robot.move_on_path_async(
            path,
            velocity=speed_mps,
            timeout_sec=timeout_s,
            lookahead=3,
            adaptive_lookahead=1,
        )
        return await task

    def cancel(self) -> None:
        self._cancel.set()

    def hover(self) -> None:
        self._apply_controls(0.0, 0.0, 1.0)
        self._connection.advance_for_sim_time(self._period)
        self._pid.reset()

    def settle(self, duration_s: float = SETTLE_DURATION_S) -> None:
        """Hold the brake while the spawned car falls onto the road.

        The scene starts paused at the authored spawn pose, which sits above
        the road, so the car has to drop before its resting height means
        anything. ``measure_resting_pose`` is what reads that height.
        """
        self._require_started()
        self._apply_controls(0.0, 0.0, 1.0)
        self._connection.advance_for_sim_time(duration_s)
        self._pid.reset()

    def measure_resting_pose(
        self,
        *,
        settle_s: float = SETTLE_DURATION_S,
        timeout_s: float = RESTING_TIMEOUT_S,
        step_s: float = 0.1,
        window_s: float = RESTING_WINDOW_S,
    ) -> WorldPose:
        """Brake through the spawn drop and return the pose the car rests at.

        The pose comes from the fast-physics state rather than a camera frame,
        because that state is exactly what a later teleport writes back. It is
        sampled until every reading in ``window_s`` agrees within
        ``RESTING_TOLERANCE_M``; if the car never quiets down inside
        ``timeout_s`` the newest reading is returned.
        """
        self.settle(settle_s)
        waited = settle_s
        window: deque = deque(maxlen=max(2, round(window_s / step_s)))
        pose = self.current_pose()
        while True:
            window.append((pose.x, pose.y, pose.z))
            spread = max(
                max(axis) - min(axis) for axis in zip(*window, strict=True)
            )
            if len(window) == window.maxlen and spread <= RESTING_TOLERANCE_M:
                return pose
            if waited >= timeout_s:
                return pose
            self._connection.advance_for_sim_time(step_s)
            waited += step_s
            pose = self.current_pose()

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

    # -- streaming port ----------------------------------------------------
    def stream_state(self) -> TimedVehicleState:
        kinematics = self._kinematics()
        pose = pose_from_kinematics(kinematics)
        timestamp_ns = int(kinematics["time_stamp"])
        previous, previous_ns = self._stream_pose, self._stream_ns
        velocity = (0.0, 0.0, 0.0)
        yaw_rate = 0.0
        if previous is not None and timestamp_ns > previous_ns:
            dt = (timestamp_ns - previous_ns) / 1e9
            velocity = (
                (pose.x - previous.x) / dt,
                (pose.y - previous.y) / dt,
                (pose.z - previous.z) / dt,
            )
            yaw_rate = (pose.yaw_deg - previous.yaw_deg) / dt
        self._stream_pose, self._stream_ns = pose, timestamp_ns
        return TimedVehicleState(
            pose, timestamp_ns, time.monotonic(), velocity, yaw_rate_deg_s=yaw_rate
        )

    def stream_velocity(
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
        state = self.stream_state()
        controls = controls_from_reference(
            self._drive_model,
            self._pid,
            velocity_world,
            yaw_rate_deg_s,
            state.velocity_world,
            state.pose.yaw_deg,
            duration_s,
        )
        self._send_controls(controls)
        self._connection.advance_for_sim_time(duration_s)

    # -- observation -------------------------------------------------------
    def capture(self, *, budget=None, timeout_s: float = 2.0) -> Observation:
        return self._observations.capture(budget=budget, timeout_s=timeout_s)

    def capture_moving(self, *, budget=None) -> Observation:
        return self._observations.capture_moving(budget=budget)

    # -- idle / reset ------------------------------------------------------
    def idle_tick(self) -> None:
        """Advance one control period with the brake held."""
        self.hover()

    def return_home(self) -> WorldPose:
        """Teleport to the settled spawn pose (verified with SetPose)."""
        pose = self._home_pose
        self.teleport(pose)
        return pose

    def set_home_pose(self, pose: WorldPose) -> None:
        """Replace the home pose with the measured resting pose.

        The constructor captures the spawn while the scene is still paused,
        which sits above the rendered road surface, so ``return_home`` would
        leave the car in the air. The session measures where the car actually
        comes to rest and records it here.
        """
        self._home_pose = WorldPose(pose.x, pose.y, pose.z, pose.yaw_deg)

    def teleport(self, pose: WorldPose) -> None:
        """Instantly move the rover and clear stale kinematics.

        Used to move an inactive rover out of the streaming view (and back).
        """
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
        # SetPose updates the fast-physics state but not the rendered actor,
        # so an inactive rover would stay visible where it was parked. Move
        # the actor explicitly with the same target pose -- unless it is
        # already there, which the server refuses (see TELEPORT_TOLERANCE_M).
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
        self._pid.reset()
        self._collisions.clear()

    def close(self) -> None:
        try:
            self._collisions.close()
            self._observations.close()
        finally:
            self._bridge.close()

    # -- internals ---------------------------------------------------------
    def _require_started(self) -> None:
        if not self._started:
            raise ExecutionError(
                "rover API control is not enabled; call start() first"
            )

    def _apply_reference(
        self,
        velocity_world: tuple[float, float, float],
        yaw_rate_deg_s: float,
        measured_velocity_world: tuple[float, float, float],
        yaw_deg: float,
        period_s: float,
    ) -> None:
        controls = controls_from_reference(
            self._drive_model,
            self._pid,
            velocity_world,
            yaw_rate_deg_s,
            measured_velocity_world,
            yaw_deg,
            period_s,
        )
        self._send_controls(controls)
        self._connection.advance_for_sim_time(period_s)

    def _send_controls(self, controls: RoverControls) -> None:
        self._apply_controls(controls.engine, controls.steering, controls.brake)

    def _apply_controls(self, engine: float, steering: float, brake: float) -> None:
        self._connection.request(
            f"{self._robot.parent_topic}/SetRoverControls",
            {
                "engine": float(engine),
                "steering_angle": float(steering),
                "brake": float(brake),
            },
        )

    def _kinematics(self) -> dict:
        return self._connection.request(
            f"{self._robot.parent_topic}/GetGroundTruthKinematics"
        )