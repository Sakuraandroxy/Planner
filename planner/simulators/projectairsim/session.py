"""A ProjectAirSim-backed dual-vehicle mission session.

One session owns the ProjectAirSim connection, the loaded ``World`` and both
actors (Rover1 and Drone1). It lives on the worker thread; nothing else may
command the vehicles while a task runs. Tasks name a vehicle id and are routed
to that vehicle's own workflow stack:

* ``BASIC_MOTION`` (drive/fly/reverse) executes directly,
* ``VISUAL_GOAL_NAVIGATION`` uses the visual-goal planner, and
* ``FREE_NAVIGATION`` follows the scene target-free until the user stops it.

The rover is armed and braked at session startup. The drone is armed and stays
at its spawn until a task targets it; ``prepare`` then performs the reference
tolerant takeoff and position hold. Between tasks the active vehicle keeps its
safe idle behaviour (drone position hold, rover brake) while ``idle_tick``
paces the loop by one control period per poll -- after scene load the engine
runs the clock itself, so this waits for it rather than stepping it.
Switching vehicles parks the previous one and hides it; ``vehicles.py`` owns
the parking bookkeeping and the altitude pin that keeps the hidden actor in
place. Only the active vehicle is visible in the streaming view.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from projectairsim import Drone, Rover

from planner.assembly import MissionInputs, VehicleArm, require_application
from ...config import AppConfig
from planner.domain import MissionResult
from planner.simulator import TaskSubmission
from planner.domain import VIEW_MODES, cycle_delta, default_view_for, validate_view_mode
from planner.domain import (
    WEATHER_CATALOG,
    WeatherCatalog,
)
from ...errors import ProtocolError
from .connection import (
    ProjectAirSimConnection,
    ProjectAirSimConnectionConfig,
)
from .drone import ProjectAirSimDrone
from .moving_source import BorrowedMovingSource
from .rover import ProjectAirSimRover
from .weather import apply_weather
# The hide bookkeeping moved to ``vehicles``; both constants stay importable
# from this module because the hide altitude is quoted off the session.
from .vehicles import (
    HIDE_ALTITUDE_GAIN_M,  # noqa: F401
    HIDE_PIN_PERIOD_S,  # noqa: F401
    VehicleRoster,
)

DEFAULT_CONTROL_PERIOD_S = 1.0 / 20.0


#: The shared assembly owns the arm type; this name stays for the session's internals.
_VehicleArm = VehicleArm


class ProjectAirSimMissionSession(VehicleRoster):
    def __init__(
        self,
        config: AppConfig,
        *,
        log: Callable[[str], None] | None = None,
        frame_sink: Callable[[np.ndarray, float], None] | None = None,
        trajectory=None,
    ):
        self._config = config
        self._log = log or (lambda message: None)
        # The console reads the remaining path from this sink. It arrives from the
        # assembly's resources, like ``frame_sink``: the session marks task
        # boundaries on it (start / clear) because it is what knows a task began
        # and ended, but it never builds one.
        self._trajectory = trajectory
        self._connection: ProjectAirSimConnection | None = None
        self._drone: ProjectAirSimDrone | None = None
        self._rover: ProjectAirSimRover | None = None
        self._arms: dict[str, _VehicleArm] = {}
        self._active_vehicle = config.server.default_vehicle
        self._init_vehicle_state()
        self._frame_sink = frame_sink
        self._publish_frames = False
        # Streaming-view cycle position; the console only ever sees the ids in
        # ``planner.domain.view``. The server never reports where the ring is,
        # and measuring it costs more than the simulator takes to boot, so the
        # position is tracked here from the declared start in
        # ``projectairsim.view_cycle_start``.
        self._view_positions: tuple[str, ...] = VIEW_MODES
        self._view_index = VIEW_MODES.index(
            config.projectairsim.view_cycle_start
        )
        self._view_mode = VIEW_MODES[self._view_index]
        self._weather: dict[str, float] | None = None
        try:
            self._connection = ProjectAirSimConnection(
                ProjectAirSimConnectionConfig(
                    address=config.projectairsim.address,
                    port_topics=config.projectairsim.port_topics,
                    port_services=config.projectairsim.port_services,
                    timeout_s=config.projectairsim.timeout_s,
                    scene_file=config.projectairsim.scene_file,
                    sim_config_path=config.projectairsim.sim_config_path,
                    delay_after_load_s=config.projectairsim.delay_after_load_s,
                )
            )
            self._connection.connect(
                wait_s=max(10.0, config.projectairsim.timeout_s)
            )
            # Pin hidden and calibration-hover vehicles after every clock
            # advance: the stepped world keeps gravity on them even while no
            # API control is active.
            self._connection.add_advance_hook(self._pin_hook)
            world = self._connection.load_scene()
            self._log(
                f"ProjectAirSim ready: scene={config.projectairsim.scene_file} "
                f"rover={config.projectairsim.rover_name} "
                f"drone={config.projectairsim.drone_name}"
            )
            self._rover = ProjectAirSimRover(
                self._connection,
                Rover(
                    self._connection.client,
                    world,
                    config.projectairsim.rover_name,
                ),
                camera=config.camera,
                depth=config.depth,
                camera_id=config.projectairsim.front_camera,
                motion=config.motion,
                frame_sink=self._sink_for("rover"),
            )
            self._drone = ProjectAirSimDrone(
                self._connection,
                Drone(
                    self._connection.client,
                    world,
                    config.projectairsim.drone_name,
                ),
                camera=config.drone.camera(config.camera),
                depth=config.depth,
                camera_id=config.projectairsim.front_camera,
                speed_mps=config.drone.speed_mps,
                motion_timeout_s=config.drone.move_timeout_s,
                takeoff_timeout_s=config.drone.takeoff_timeout_s,
                land_timeout_s=config.drone.land_timeout_s,
                readiness_stable_s=config.drone.readiness_stable_s,
                readiness_poll_interval_s=(
                    config.drone.readiness_poll_interval_s
                ),
                takeoff_climb_m=config.drone.takeoff_climb_m,
                motion=config.motion,
                tracking=config.continuous,
                frame_sink=self._sink_for("drone"),
            )
            self._rover.start()
            self._drone.start()
            # The arms are assembled by the registry once this session has
            # handed over its ports; the boot below only needs the vehicles.
            # The scene starts paused with both robots on the same ground
            # point, so one of them has to leave before the first clock advance
            # or the two colliders shove each other off the spawn.
            #
            # For the rover's home measurement below that vehicle is the drone,
            # not "the inactive one": the measurement is only meaningful with
            # the car on the road, whatever the default vehicle is.
            self._hide_quiet("drone")
            self._publish_frames = True
            try:
                try:
                    # One warm-up capture gives idle viewers an initial frame.
                    self._rover.capture(
                        timeout_s=min(8.0, config.projectairsim.timeout_s)
                    )
                except Exception as error:
                    self._log(
                        f"initial preview frame unavailable: {error}", "WARNING"
                    )
                try:
                    # The authored spawn sits above the road, so ``return_home``
                    # needs the height the car actually rests at.
                    pose = self._rover.measure_resting_pose()
                except Exception as error:
                    self._log(
                        f"could not measure the rover home pose: {error}",
                        "WARNING",
                    )
                else:
                    self._rover.set_home_pose(pose)
                    self._log(
                        "Rover home pose measured: "
                        f"x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} "
                        f"yaw={pose.yaw_deg:.1f}"
                    )
            finally:
                self._publish_frames = False
            # Back to the shape the default vehicle expects. The vehicle that
            # leaves goes up before the vehicle that arrives comes down: they
            # share one spawn point, so the other order parks one robot on top
            # of the other.
            self._hide_inactive_at_startup()
            if self._active_vehicle != "rover":
                self._unhide_quiet("drone")
            self._apply_initial_view()
            # Everything above ran against a paused clock, which is what keeps
            # the load transient out: physics during scene load dropped the
            # drone from 38 m and broke the four-camera calibration. With the
            # vehicles spawned, hidden, settled and measured, hand the clock
            # back to the engine so a slow caller cannot freeze the picture.
            if self._connection.resume_clock():
                self._log(
                    "Simulator clock runs free; the picture keeps moving while "
                    "the planner is busy"
                )
            else:
                self._log(
                    "Simulator clock stays stepped; each control period is one "
                    "explicit advance",
                    "WARNING",
                )
        except Exception:
            self.close()
            raise

    # -- workflow assembly ------------------------------------------------
    def mission_inputs(self) -> MissionInputs:
        """Hand the assembly the engine-side objects it wires pipelines around.

        The task pipeline does not know which simulator owns the vehicle, so it
        lives in ``planner/assembly/mission_assembly.py``. This session says
        which vehicles ProjectAirSim has and which moving observation source
        belongs to them; it does not build planners, detectors or telemetry
        sinks -- a peer middleware may not construct its peer.
        """
        return MissionInputs(
            vehicles={"rover": self._rover, "drone": self._drone},
            moving_source_factory=BorrowedMovingSource,
        )

    def attach_arms(self, arms: dict) -> None:
        """Accept the assembled pipelines the registry built for these vehicles."""
        missing = set(self._config.server.vehicles) - set(arms)
        if missing:
            raise ProtocolError(
                f"mission assembly returned no arm for {sorted(missing)}"
            )
        self._arms = dict(arms)

    # -- ports -------------------------------------------------------------
    # ``return_home`` and ``activate_vehicle`` arrive with ``VehicleRoster``:
    # every step in them moves a vehicle.
    def run(
        self,
        task: TaskSubmission,
        *,
        stop_event=None,
        log: Callable[[str], None] | None = None,
    ) -> MissionResult:
        vehicle_id = task.vehicle or self._config.server.default_vehicle
        arm = self._arms.get(vehicle_id)
        if arm is None:
            raise ProtocolError(f"unknown vehicle '{vehicle_id}'")
        application = require_application(arm)
        event_sink = log or self._log
        if vehicle_id != self._active_vehicle:
            previous = self._active_vehicle
            self._park(previous, event_sink)
            try:
                self._hide(previous)
            except Exception as error:
                event_sink(
                    f"could not move the {previous} out of view: "
                    f"{type(error).__name__}: {error}",
                    "WARNING",
                )
            self._unhide(vehicle_id)
            self._active_vehicle = vehicle_id
        else:
            self._active_vehicle = vehicle_id
        if stop_event is not None:
            def _hold():
                if stop_event.is_set():
                    raise KeyboardInterrupt

            arm.execution.bind_hold_checkpoint(_hold)
        else:
            arm.execution.bind_hold_checkpoint(None)
        self._publish_frames = True
        try:
            if self._trajectory is not None:
                self._trajectory.start(task.instruction)
            plan = application.prepare(task.instruction)
            event_sink(
                f"{vehicle_id} mission plan: "
                + ", ".join(
                    f"{stage.stage_id}:{stage.kind.value}"
                    for stage in plan.stages
                )
            )
            self._prepare_vehicle(arm, event_sink)
            return application.execute_plan(plan)
        except KeyboardInterrupt:
            return MissionResult(
                False, (), error="cancelled: task was cancelled"
            )
        finally:
            self._publish_frames = False
            arm.execution.bind_hold_checkpoint(None)
            if self._trajectory is not None:
                self._trajectory.clear()
            self._settle(arm, event_sink)

    def idle_tick(self) -> None:
        """Give the world one control period on the owner thread."""
        if self._active_vehicle == "drone" and self._drone is not None:
            self._drone.idle_tick()
        elif self._rover is not None:
            self._rover.idle_tick()
        elif self._connection is not None:
            self._connection.advance_for_sim_time(DEFAULT_CONTROL_PERIOD_S)

    # -- weather / view ----------------------------------------------------
    def weather_catalog(self) -> WeatherCatalog:
        """Static field/preset catalog (does not change world state)."""
        return WEATHER_CATALOG

    @property
    def view_mode(self) -> str:
        return self._view_mode

    @property
    def weather(self) -> dict[str, float] | None:
        return dict(self._weather) if self._weather is not None else None

    def set_weather(self, fields) -> dict[str, float]:
        """Apply a complete eight-field weather state to the world."""
        values = apply_weather(self._connection, fields, self._log)
        self._weather = values
        return dict(values)

    def set_view_mode(self, mode: str) -> str:
        """Cycle the streaming view forward to the requested camera.

        The position is arithmetic from the declared cycle start, because the
        server cannot say where the ring is and measuring it costs more than
        the simulator takes to boot.
        """
        mode = validate_view_mode(mode)
        if self._connection is None or self._connection.world is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        positions = self._view_positions
        target = positions.index(mode)
        start = positions[self._view_index]
        steps = cycle_delta(self._view_index, target)
        for _ in range(steps):
            self._switch_view_once()
        self._view_index = target
        self._view_mode = mode
        self._log(
            f"Streaming view: {mode} ({steps} switch(es) from {start})."
        )
        return mode

    def _switch_view_once(self) -> None:
        """Walk the streaming cycle one camera forward."""
        if self._connection is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        if not self._connection.switch_streaming_view():
            raise ProtocolError("simulator refused to switch the streaming view")

    # -- internals ---------------------------------------------------------
    def _prepare_vehicle(self, arm: _VehicleArm, log) -> None:
        if arm.is_drone:
            self._drone.prepare(takeoff_if_landed=True)
            log("Drone airborne; position hold confirmed.")
        else:
            # start() already armed and braked the rover.
            self._rover.hold()

    def _apply_initial_view(self) -> None:
        """Park the streaming view on the active vehicle's chase (tail) camera.

        Runs once the scene is loaded, so the console does not open on whichever
        camera the engine happened to pick -- regularly the vehicle that
        ``_hide_inactive_at_startup`` lifted out of view.
        """
        try:
            self.set_view_mode(default_view_for(self._active_vehicle))
        except Exception as error:
            self._log(
                "could not switch to the initial streaming view: "
                f"{type(error).__name__}: {error}",
                "WARNING",
            )

    def _settle(self, arm: _VehicleArm, log) -> None:
        """Keep the reference post-task hold; land the drone if it fails."""
        try:
            if arm.is_drone:
                self._drone.hold_steady(require_airborne=False)
            else:
                self._rover.hold()
        except Exception as error:
            log(
                f"post-task hold failed: {type(error).__name__}: {error}",
                "WARNING",
            )
            if arm.is_drone:
                try:
                    if not self._drone.land():
                        log("post-task emergency land was not confirmed.", "ERROR")
                except Exception as land_error:
                    log(
                        "post-task emergency land failed: "
                        f"{type(land_error).__name__}: {land_error}",
                        "ERROR",
                    )

    def _sink_for(
        self, vehicle_id: str
    ) -> Callable[[np.ndarray, float], None]:
        """A frame sink that forwards only the active vehicle's camera.

        Both vehicles' front-camera topic streams subscribe at startup and push
        every frame to this session's shared hub. An unfiltered sink lets the
        hidden vehicle -- parked 1000 m up with its camera aimed at the sky --
        overwrite the live preview, so the console "相机画面" flickers between
        ground and sky. Gating on the active vehicle keeps the preview on the
        camera its label promises ("当前载具的前视相机"); the inactive vehicle
        keeps streaming for its own observations, which never go through here.
        """

        def sink(frame: np.ndarray, timestamp_s: float) -> None:
            if (
                vehicle_id == self._active_vehicle
                and self._frame_sink is not None
                and self._publish_frames
            ):
                self._frame_sink(frame, timestamp_s)

        return sink

    def close(self, *, skip_landing: bool = False) -> None:
        try:
            for arm in self._arms.values():
                for workflow in arm.workflows:
                    try:
                        workflow.close()
                    except Exception:
                        pass
                if arm.model_calls is not None:
                    try:
                        arm.model_calls.close()
                    except Exception:
                        pass
            self._arms = {}
            if self._drone is not None:
                # A torn-down simulator can make the landed-state query raise
                # ("Object closed"); treat that as "landing cannot be
                # confirmed" and keep closing. The connection release below
                # must run even when this fails, otherwise the process-wide
                # single-consumer guard stays claimed and every reconnect is
                # rejected with "another ProjectAirSim connection is active".
                try:
                    landed = self._drone.landed
                except Exception as error:
                    landed = True
                    self._log(
                        "could not confirm the drone landing state on close: "
                        f"{type(error).__name__}: {error}",
                        "WARNING",
                    )
                if not landed:
                    if skip_landing:
                        self._log(
                            "Skipping drone landing: the scene is being replaced."
                        )
                    else:
                        try:
                            self._drone.land()
                        except Exception as error:
                            self._log(
                                f"drone landing on close failed: {error}",
                                "WARNING",
                            )
                try:
                    self._drone.close()
                except Exception:
                    pass
                self._drone = None
            if self._rover is not None:
                try:
                    self._rover.close()
                except Exception:
                    pass
                self._rover = None
        finally:
            if self._connection is not None:
                try:
                    self._connection.remove_advance_hook(self._pin_hook)
                except Exception:
                    pass
                try:
                    self._connection.close()
                except Exception:
                    pass
                self._connection = None