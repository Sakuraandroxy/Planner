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

from ...config import CameraSettings, DepthSettings
from ...domain.observation import Observation
from ...domain.pose import WorldPose
from ...domain.streaming import TimedVehicleState
from ...errors import ExecutionError
from ...services.segment_motion import MotionLimits
from .collision import CollisionEvent, CollisionMonitor
from .connection import ProjectAirSimConnection
from .coordinates import pose_from_kinematics
from .drive import (
    RoverDriveModel,
    RoverControls,
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

# ``SetObjectPose`` refuses ("Unable to move object ... not movable") when the
# actor is already at the requested pose. That happens whenever a vehicle is
# homed right after being restored from its hidden parking spot, because both
# moves target the same pose. The fast-physics state is authoritative and the
# rendered actor syncs on the next clock advance, so a teleport to where the
# rover already is can skip the actor move instead of failing.
TELEPORT_TOLERANCE_M = 0.01
TELEPORT_TOLERANCE_DEG = 0.5


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
    ):
        self._connection = connection
        self._robot = robot
        self._motion = motion or MotionLimits()
        self._tracking = tracking_limits(self._motion)
        self._drive_model = drive_model
        self._pid = SpeedPid(drive_model)
        self._period = 1.0 / self._tracking.control_hz
        self._cancel = threading.Event()
        self._started = False
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
        self._last_collision: CollisionEvent | None = None
        # Home pose used by ``return_home``. The raw spawn pose read here can
        # be above the rendered resting pose, so the session replaces it with
        # the measured pose from the first captured frame.
        self._home_pose = self.current_pose()
        self._tracker = SteppedMotionTracker(
            self._apply_reference,
            self.current_pose,
            self._tracking,
            motion_timeout_s,
            cancelled=self._cancel.is_set,
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

    def cancel(self) -> None:
        self._cancel.set()

    def hover(self) -> None:
        self._apply_controls(0.0, 0.0, 1.0)
        self._connection.advance_for_sim_time(self._period)
        self._pid.reset()

    def settle(self, duration_s: float = SETTLE_DURATION_S) -> None:
        """Hold the brake while the spawned car falls onto the road.

        The scene starts paused with the authored spawn pose, which sits above
        the rendered road surface; the first camera frame is captured before
        that initial drop finishes. Stepping the clock with the brake held
        yields the height the car actually rests at, which is the pose
        ``return_home`` must teleport to.
        """
        self._require_started()
        self._apply_controls(0.0, 0.0, 1.0)
        self._connection.advance_for_sim_time(duration_s)
        self._pid.reset()

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
        if previous is not None and timestamp_ns > previous_ns:
            dt = (timestamp_ns - previous_ns) / 1e9
            velocity = (
                (pose.x - previous.x) / dt,
                (pose.y - previous.y) / dt,
                (pose.z - previous.z) / dt,
            )
        self._stream_pose, self._stream_ns = pose, timestamp_ns
        return TimedVehicleState(pose, timestamp_ns, time.monotonic(), velocity)

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
        """Replace the home pose with a measured (rendered) spawn pose.

        The constructor captures the spawn while the scene is still paused;
        on fast-physics rovers that pose can sit above the rendered resting
        pose, which makes ``return_home`` leave the car floating. The session
        measures the rendered pose from the first camera frame and records it
        here.
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
        self._stream_pose, self._stream_ns = None, 0
        self._pid.reset()
        self._collisions.clear()

    def close(self) -> None:
        self._collisions.close()
        self._observations.close()

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