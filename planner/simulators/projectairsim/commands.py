"""ProjectAirSim drone command downlink: velocity, native tasks, pose sets.

Every command the drone adapter sends is encoded here: the streaming velocity
exchange, the pump that steps the simulator clock while a native takeoff or
land task runs, and the ``SetPose``/``SetObjectPose`` pair that makes a
teleport authoritative. The drone decides what to fly and when; this module
owns how a command is sent and when its result counts as complete.
"""

from __future__ import annotations

import asyncio
import logging
import math

from projectairsim.drone import YawControlMode

from planner.domain import WorldPose
from ...errors import ExecutionError
from .async_bridge import AsyncBridge
from .connection import ProjectAirSimConnection
from .coordinates import pose_to_transform

logger = logging.getLogger(__name__)

# ``SetObjectPose`` refuses ("Unable to move object ... not movable") when the
# actor is already at the requested pose. That happens whenever a vehicle is
# homed right after being restored from its hidden parking spot, because both
# moves target the same pose. The fast-physics state is authoritative and the
# rendered actor syncs on the next clock advance, so a teleport to where the
# drone already is can skip the actor move instead of failing.
TELEPORT_TOLERANCE_M = 0.01
TELEPORT_TOLERANCE_DEG = 0.5


class ProjectAirSimCommands:
    def __init__(
        self,
        connection: ProjectAirSimConnection,
        robot,
        bridge: AsyncBridge,
    ):
        self._connection = connection
        self._robot = robot
        self._bridge = bridge

    # -- velocity ----------------------------------------------------------
    def move_by_velocity(
        self,
        velocity_world: tuple[float, float, float],
        yaw_rate_deg_s: float,
        duration_s: float,
    ) -> None:
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

    # -- native tasks ------------------------------------------------------
    def run_native(
        self,
        command,
        *,
        step_s: float,
        deadline_s: float,
        timeout_s: float,
    ):
        """Run one native coroutine, stepping the clock until it completes."""
        return self._bridge.run(
            self._timed_task(command, step_s=step_s, deadline_s=deadline_s),
            timeout_s=timeout_s,
        )

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

    # -- pose --------------------------------------------------------------
    def set_pose(self, pose: WorldPose, *, current: WorldPose) -> None:
        """Move the fast-physics pose and the rendered actor to ``pose``."""
        transform = pose_to_transform(pose)
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
