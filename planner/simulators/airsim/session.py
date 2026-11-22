"""AirSim (UE4) mission session: one owner thread's side of a UE4 stack.

The UE5 counterpart is ``planner/simulators/projectairsim/session.py``. Both build
their own vehicles and then hand them to the same wiring in
``planner/assembly/mission_assembly.py``, so a task means the same thing on either
engine and neither simulator package imports the other.

Three pieces here are AirSim's own, and come from the old branch's ``build_runtime``
rather than from a fresh idea:

* the control connection owns the vehicle, while a capture that runs on another
  thread gets its own connection (``capture_worker_factory``) -- one msgpack RPC
  transport is not shared between the flight loop and a frame reader;
* ``ObservationGate`` is what joins them: a frame is only taken when the flight loop
  has left the vehicle in a state worth photographing;
* the position hold is bound into the execution checkpoints, so a planned segment
  cannot start while the aircraft is still settling.

The session runs on the worker thread and owns the connection for its whole life:
``AirSimConnection`` says so in its own docstring, and the close path is what honours
that claim.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from planner.assembly import MissionInputs, require_application
from planner.domain import MissionResult
from planner.domain import TaskSubmission
from planner.domain import WEATHER_CATALOG, validate_weather_values
from planner.errors import ProtocolError
from planner.runtime import ExecutionService

from .connection import AirSimConnection
from .observation_gate import ObservationGate
from .observation_source import AirSimObservationSource
from .runtime import WEATHER_FIELDS, WEATHER_PRESETS, build_ports
from .server_vehicle import ServerVehicle
from .weather import AirSimWeather


class AirSimMissionSession:
    """The UE4 side of the worker protocol: run, close, and the idle heartbeat."""

    #: How close to the recorded ground height counts as touchdown (metres).
    #: AirSim's own ``landed_state`` is not usable here: measured on Blocks it stays
    #: at 1 (Flying) after a land that returned True and the vehicle is back on its
    #: spawn height.
    TOUCHDOWN_TOLERANCE_M = 0.25

    def __init__(
        self,
        config,
        family,
        *,
        log: Callable[[str, str], None],
    ):
        self._config = config
        self._log = log
        self._closed = False
        #: Height the vehicle stood on before the first takeoff, in UE coordinates.
        #: ``close`` lands back onto it; None means this session never flew.
        self._ground_z: float | None = None
        self.connection = AirSimConnection(
            family.host,
            family.ports["rpc"],
            family.timeout_s,
            # A motion call outlives a state read by design: the flight task blocks
            # until the aircraft settles.
            motion_timeout_s=config.drone.move_timeout_s + 2,
            vehicle_name=family.vehicle_name,
        )
        # ``config.motion`` 就是跟踪侧那份包络：位置保持、运动跟踪器与段规划器
        # 读同一组字段，这里不再转换。
        limits = config.motion
        self.server = ServerVehicle(
            self.connection,
            limits,
            config.drone.move_timeout_s,
            readiness_stable_s=config.drone.readiness_stable_s,
            readiness_poll_interval_s=config.drone.readiness_poll_interval_s,
            speed_mps=config.drone.speed_mps,
            tracking=config.continuous,
            takeoff_climb_m=config.drone.takeoff_climb_m,
        )
        control = self.server.control

        self.execution = ExecutionService(control)
        self.execution.bind_hold_checkpoint(self.server.holding.tick)
        self.gate = ObservationGate(
            control,
            limits,
            config.drone.move_timeout_s,
            owner_thread_id=threading.get_ident(),
            check_safety=self.execution.check_safety,
        )
        self.execution.bind_observation_checkpoint(
            self.gate.service_requests, self.gate.cancel_requests
        )

        self.observation = AirSimObservationSource(
            self.connection,
            control,
            family.camera_id,
            config.depth.min_m,
            config.depth.max_m,
            observation_gate=self.gate,
            verify_image_pair=True,
            depth_mapping=config.depth.mapping,
            camera_mount_position_body_m=config.drone.camera_offset_m,
            camera_mount_rpy_deg=config.drone.camera_mount_rpy_deg,
        )
        #: Transports handed to capture workers; each is closed with the session.
        self._capture_connections: list[AirSimConnection] = []

        def build_capture_source() -> AirSimObservationSource:
            read = AirSimConnection(
                family.host, family.ports["rpc"], family.timeout_s,
                vehicle_name=family.vehicle_name,
            )
            self._capture_connections.append(read)
            return AirSimObservationSource(
                read,
                None,
                family.camera_id,
                config.depth.min_m,
                config.depth.max_m,
                verify_image_pair=True,
                depth_mapping=config.depth.mapping,
                camera_mount_position_body_m=config.drone.camera_offset_m,
                camera_mount_rpy_deg=config.drone.camera_mount_rpy_deg,
            )

        #: One read-only transport per capture worker, including moving capture.
        self.build_capture_source = build_capture_source
        self.observation.capture_worker_factory = build_capture_source

        self.weather = AirSimWeather(self.connection, WEATHER_FIELDS, WEATHER_PRESETS)
        self.ports = build_ports(
            vehicle=control,
            observation=self.observation,
            weather=self.weather,
        )

    def mission_inputs(self) -> MissionInputs:
        """Hand the assembly the engine-side objects it wires pipelines around.

        This session used to build the planner stack itself -- ``detector``,
        ``planner``, ``geometry`` and the telemetry sink arrived as constructor
        arguments and it called ``build_vehicle_arm`` in ``__init__``. Two peer
        middlewares do not construct each other: the assembly does that, from
        here and from ``attach_arms``.
        """
        return MissionInputs(
            vehicles={"drone": self.server.control},
            execution=self.execution,
            # AirSim keeps the two roles apart on purpose: the vehicle owns the
            # control connection, and the observation source owns a second one
            # for pixels.
            observation_source=self.observation,
            # Moving observations run on a worker thread. Handing them the
            # control connection put every frame request behind its lock, and
            # ``simGetImages`` on this build holds it for 0.4-0.5 s: measured in
            # a planner-driven flight, sixteen control ticks of 66 s waited that
            # long, the 0.2 s velocity command expired in the gap, and the drone
            # braked and re-accelerated through every planning round. AirSim's
            # clock keeps running during that wait; ProjectAirSim's stepped clock
            # does not, which is why only UE4 shook. A read-only transport per
            # capture worker keeps pixels off the control lock.
            moving_source_factory=lambda vehicle: self.build_capture_source(),
        )

    def attach_arms(self, arms: dict) -> None:
        """Accept the assembled pipeline the registry built for these ports."""
        if "drone" not in arms:
            raise ProtocolError(
                f"mission assembly returned no drone arm, got {sorted(arms)}"
            )
        self._arm = arms["drone"]
        if self._arm.loop is not None:
            # 段执行每拍经取帧门服务在途请求；恢复后从实测位姿重锚。
            self._arm.loop.observation_gate = self.gate

    # -- worker protocol ---------------------------------------------------
    def run(
        self,
        task: TaskSubmission,
        *,
        stop_event=None,
        log: Callable[[str], None] | None = None,
    ) -> MissionResult:
        vehicle_id = task.vehicle or "drone"
        if vehicle_id != "drone":
            raise ProtocolError(
                f"AirSim drives a drone only; asked for vehicle '{vehicle_id}'"
            )
        application = require_application(self._arm)
        event_sink = log or (lambda message, level="INFO": self._log(message, level))

        def _hold():
            if stop_event is not None and stop_event.is_set():
                raise KeyboardInterrupt

        self.execution.bind_hold_checkpoint(_hold)
        try:
            plan = application.prepare(task.instruction)
            event_sink(
                "drone mission plan: "
                + ", ".join(f"{stage.stage_id}:{stage.kind.value}" for stage in plan.stages)
            )
            # Airborne before anything executes: the UE4 flight paths assume hover
            # authority from the first segment, and readiness is measured rather
            # than assumed -- the same rule the old branch's ServerVehicle encodes.
            # The ground reference is the highest z this vehicle has ever stood at:
            # UE points z downward, so the floor is the maximum, and taking the max
            # keeps the reference honest when a session is built while the drone is
            # already airborne after an earlier one.
            z_now = self.server.control.current_pose().z
            if self._ground_z is None or z_now > self._ground_z:
                self._ground_z = z_now
            self.server.prepare(takeoff_if_landed=True, ground_z=self._ground_z)
            event_sink("Drone airborne; position hold confirmed.")
            return application.execute_plan(plan)
        except KeyboardInterrupt:
            return MissionResult(False, (), error="cancelled: task was cancelled")
        finally:
            self.execution.bind_hold_checkpoint(self.server.holding.tick)

    def idle_tick(self) -> None:
        """Keep the hold alive between tasks so the drone does not sink."""
        if self.server.holding.active:
            self.server.holding.tick()

    def weather_catalog(self) -> object:
        return WEATHER_CATALOG

    def set_weather(self, fields) -> dict[str, float]:
        values = validate_weather_values(fields)
        applied = self.weather.apply(values)
        self._log(f"Weather applied: {applied}")
        return dict(applied)

    def close(self, *, skip_landing: bool = False) -> None:
        """Land, confirm the touchdown, then release every transport.

        ``skip_landing`` is what a map switch passes: a scene about to be reloaded
        has no use for a landing animation, and waiting for one would hold the
        switch open.

        The landing itself cannot be left to :meth:`AirSimConnection.shutdown`: that
        method deliberately refuses to disarm without positive landed evidence, and a
        task that ends on a hover leaves the aircraft airborne with
        ``landed_state == 1`` -- measured here, the first UE4 task ended at z=-0.83 m.
        So this lands it and waits for the height to come back to the ground record.
        """
        if self._closed:
            return
        self._closed = True
        try:
            settled = False
            if not skip_landing and self._ground_z is not None:
                try:
                    settled = self._land_and_confirm()
                except Exception as error:
                    self._log(f"landing before close failed: {error}", "WARNING")
                else:
                    # The measurement above is the evidence; landed_state on this
                    # build stays at 1 even after a confirmed touchdown, so without
                    # it the shutdown would leave every landed drone armed.
                    try:
                        self.connection.shutdown(landed_on_ground=settled)
                    except Exception as error:
                        self._log(f"vehicle shutdown was incomplete: {error}", "WARNING")
        finally:
            for read in self._capture_connections:
                try:
                    read.close_transport()
                except Exception as error:
                    self._log(
                        f"capture transport did not close cleanly: {error}", "WARNING"
                    )
            self._capture_connections.clear()
            self.connection.close_transport()

    def _land_and_confirm(self, timeout_s: float = 60.0) -> bool:
        """Descend onto the recorded ground height; return whether it was reached."""
        self.connection.call_async_and_wait("landAsync")
        deadline = time.monotonic() + timeout_s
        z = self.server.control.current_pose().z
        settled = z >= self._ground_z - self.TOUCHDOWN_TOLERANCE_M
        while not settled and time.monotonic() < deadline:
            time.sleep(0.5)
            z = self.server.control.current_pose().z
            settled = z >= self._ground_z - self.TOUCHDOWN_TOLERANCE_M
        if settled:
            self._log(
                f"Vehicle on the ground: z={z:.2f} against the ground record "
                f"{self._ground_z:.2f}."
            )
        else:
            self._log(
                f"touchdown not confirmed in {timeout_s:g}s: z={z:.2f} against a "
                f"ground record of {self._ground_z:.2f}; leaving the decision to the "
                "safe shutdown",
                "WARNING",
            )
        return settled
