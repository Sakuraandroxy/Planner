"""A ProjectAirSim-backed dual-vehicle mission session.

One session owns the ProjectAirSim connection, the paused ``World`` and both
actors (Rover1 and Drone1). It lives on the worker thread; nothing else may
tick the world while a task runs. Tasks name a vehicle id and are routed to
that vehicle's own workflow stack:

* ``BASIC_MOTION`` (drive/fly/reverse) executes directly,
* ``VISUAL_GOAL_NAVIGATION`` uses the visual-goal planner, and
* ``FREE_NAVIGATION`` follows the scene target-free until the user stops it.

The rover is armed and braked at session startup. The drone is armed but stays
landed until a task targets it; ``prepare`` then performs the reference
tolerant takeoff and position hold. Between tasks the active vehicle keeps its
safe idle behaviour (drone position hold, rover brake) while ``idle_tick``
advances the paused world. Switching vehicles parks the previous one, which
lands the drone before another vehicle's task starts moving the world, and
teleports it high above the map: ProjectAirSim has no runtime robot despawn
API, so "hiding" the inactive vehicle is a far-away teleport. A horizontal
teleport used to drop the drone inside the Neighbourhood wall mesh, where it
generated thousands of contacts per minute, so the hide spot is straight up
and the vehicle is pinned there while the world is stepped (gravity would
otherwise pull it back down). Only the active vehicle is visible in the
streaming view.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, MutableMapping

import numpy as np

from projectairsim import Drone, Rover
from projectairsim.types import WeatherParameter

from ...application.application import PlannerApplication
from ...application.factories import (
    build_completion_checker,
    build_free_navigation_planner,
    build_planner,
    build_task_parser,
    build_trace_factory,
)
from ...application.mission_runner import MissionRunner
from ...application.mission_validator import MissionValidator
from ...application.workflow_router import WorkflowRouter
from ...capabilities.approach import TargetApproach
from ...capabilities.completion.visual_goal import VisualGoalCompletion
from ...capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from ...capabilities.task_validation.basic_motion import (
    BasicMotionStageValidator,
)
from ...capabilities.task_validation.free_navigation import (
    FreeNavigationStageValidator,
)
from ...capabilities.task_validation.visual_goal_navigation import (
    VisualGoalStageValidator,
)
from ...config import AppConfig
from ...domain.mission import TaskKind
from ...domain.pose import WorldPose
from ...domain.result import MissionResult
from ...domain.server import TaskSubmission
from ...domain.view import (
    INITIAL_STREAM_INDEX,
    VIEW_MODES,
    cycle_delta,
    default_view_for,
    validate_view_mode,
)
from ...domain.weather import (
    WEATHER_CATALOG,
    WEATHER_FIELDS,
    WeatherCatalog,
    validate_weather_values,
)
from ...errors import ExecutionError, ProtocolError
from ...services.execution import ExecutionService
from ...services.model_calls import ModelCallService
from ...services.motion import SynchronizedMotionPlanner
from ...services.trajectory_validation import TrajectoryValidator
from ...workflows.basic_motion import BasicMotionWorkflow
from ...workflows.free_navigation import FreeNavigationWorkflow
from ...workflows.visual_goal_navigation import VisualGoalNavigationWorkflow
from ..projectairsim.connection import (
    ProjectAirSimConnection,
    ProjectAirSimConnectionConfig,
)
from ..projectairsim.drone import ProjectAirSimDrone
from ..projectairsim.moving_source import BorrowedMovingSource
from ..projectairsim.rover import ProjectAirSimRover
from ..projectairsim.streaming_views import (
    calibrate_streaming_views,
    service_frame_fetcher,
    verify_streaming_views,
    xpra_screenshot,
)

DEFAULT_CONTROL_PERIOD_S = 1.0 / 20.0

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


@dataclass
class _VehicleArm:
    vehicle_id: str
    vehicle: object
    is_drone: bool
    execution: ExecutionService
    application: PlannerApplication | None
    model_calls: ModelCallService | None
    workflows: tuple


class ProjectAirSimMissionSession:
    def __init__(
        self,
        config: AppConfig,
        *,
        detector,
        planner,
        geometry,
        log: Callable[[str], None] | None = None,
        frame_sink: Callable[[np.ndarray, float], None] | None = None,
        trajectory=None,
        view_cache: MutableMapping[str, tuple[str, ...]] | None = None,
    ):
        self._config = config
        self._detector = detector
        self._planner = planner
        self._geometry = geometry
        self._trajectory = trajectory
        self._view_cache = view_cache
        self._log = log or (lambda message: None)
        self._connection: ProjectAirSimConnection | None = None
        self._drone: ProjectAirSimDrone | None = None
        self._rover: ProjectAirSimRover | None = None
        self._arms: dict[str, _VehicleArm] = {}
        self._active_vehicle = config.server.default_vehicle
        # Parked pose of each vehicle currently teleported out of view, and
        # the high-altitude pose it is pinned at while the world steps.
        self._hidden_poses: dict[str, WorldPose] = {}
        self._hidden_teleports: dict[str, WorldPose] = {}
        self._last_pin_ns: int | None = None
        self._pin_failures: set[str] = set()
        # Keep one bound-method object so add/remove refer to the same hook.
        self._pin_hook = self._pin_hidden
        self._frame_sink = frame_sink
        self._publish_frames = False
        # Streaming-view cycle position; the console only ever sees the ids in
        # ``planner.domain.view``. ``_view_positions`` maps a cycle position to
        # a view mode and is measured at runtime because the server's cycle
        # order is not stable across builds or scene loads.
        self._view_positions: tuple[str, ...] = VIEW_MODES
        self._view_index = INITIAL_STREAM_INDEX
        self._view_mode = VIEW_MODES[INITIAL_STREAM_INDEX]
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
                motion_timeout_s=config.drone.move_timeout_s,
                frame_sink=self._sink,
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
                motion=config.motion,
                tracking=config.continuous,
                frame_sink=self._sink,
            )
            self._rover.start()
            self._drone.start()
            self._build_arms()
            # The scene starts paused with both robots on the same ground
            # point. Move the inactive one out of view before the first clock
            # advance so the two colliders never touch and every streaming
            # camera looks at a different part of the map during calibration.
            self._hide_inactive_at_startup()
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
                    # The authored spawn sits above the rendered road surface
                    # and the first frame is captured before the car has
                    # fallen onto it. Brake through the drop and measure the
                    # settled pose so ``return_home`` does not leave the car
                    # in the air.
                    self._rover.settle()
                    observation = self._rover.capture(
                        timeout_s=min(8.0, config.projectairsim.timeout_s)
                    )
                except Exception as error:
                    self._log(
                        f"could not measure the rover home pose: {error}",
                        "WARNING",
                    )
                else:
                    self._rover.set_home_pose(observation.pose)
                    self._log(
                        "Rover home pose measured: "
                        f"x={observation.pose.x:.2f} "
                        f"y={observation.pose.y:.2f} "
                        f"z={observation.pose.z:.2f} "
                        f"yaw={observation.pose.yaw_deg:.1f}"
                    )
            finally:
                self._publish_frames = False
            self._calibrate_views()
        except Exception:
            self.close()
            raise

    # -- workflow assembly ------------------------------------------------
    def _build_arms(self) -> None:
        config = self._config
        free_planner = build_free_navigation_planner(config)
        aerial_free_planner = build_free_navigation_planner(
            config, aerial=True
        )
        # The drone keeps the reference Planner's UAV prompts; the rover keeps
        # carla-vln's ground-vehicle prompts. Geometric mode has no prompt, so
        # it reuses the already-built planner.
        aerial_planner = (
            build_planner(config, config.server.planner, aerial=True)
            if config.server.planner == "vlm"
            else self._planner
        )
        if free_planner is None:
            self._log(
                "free navigation planner is not configured; "
                "free_navigation stages will be rejected",
                "WARNING",
            )
        completion_checker = build_completion_checker(config)
        trace_factory = build_trace_factory(
            config, detector=self._detector, planner=self._planner
        )
        self._arms["rover"] = self._build_arm(
            "rover",
            self._rover,
            self._planner,
            free_planner,
            completion_checker,
            trace_factory,
        )
        self._arms["drone"] = self._build_arm(
            "drone",
            self._drone,
            aerial_planner,
            aerial_free_planner,
            completion_checker,
            trace_factory,
        )

    def _build_arm(
        self,
        vehicle_id: str,
        vehicle,
        planner,
        free_planner,
        completion_checker,
        trace_factory,
    ) -> _VehicleArm:
        config = self._config
        settings = config.visual_goal_navigation
        limits = settings.limits
        execution = ExecutionService(vehicle)
        motion = SynchronizedMotionPlanner(
            config.motion.max_speed_mps, vehicle.tracking_limits
        )
        goal_observer = BBoxGoalObserver(
            self._detector,
            min(
                config.target_detection.timeout_s,
                limits.model_request_timeout_s,
            ),
        )
        validator = TrajectoryValidator(
            config.trajectory_planner.point_count,
            config.trajectory_planner.max_step_m,
            config.trajectory_planner.max_yaw_step_deg,
        )
        visual = VisualGoalNavigationWorkflow(
            vehicle,
            goal_observer,
            planner,
            validator,
            motion,
            execution,
            self._geometry,
            VisualGoalCompletion(limits, TargetApproach()),
            limits,
            trace_factory=trace_factory,
            completion_checker=completion_checker,
            trigger_rules=settings.completion_rules,
            sliding_window=config.sliding_window,
            motion_limits=vehicle.tracking_limits,
            collision_check=(
                vehicle.collision_state
                if settings.collision_check_enabled
                else None
            ),
            moving_observation_source=(
                BorrowedMovingSource(vehicle)
                if config.sliding_window.enabled
                else None
            ),
            stream_speed_mps=config.motion.max_speed_mps,
            stream_waypoint_timeout_s=60.0,
            continuous_tracking=config.continuous,
            telemetry=self._trajectory,
        )
        basic = BasicMotionWorkflow(
            vehicle,
            motion,
            execution,
            limits=vehicle.tracking_limits,
            telemetry=self._trajectory,
        )
        workflows = {
            TaskKind.BASIC_MOTION: basic,
            TaskKind.VISUAL_GOAL_NAVIGATION: visual,
        }
        stage_validators = {
            TaskKind.BASIC_MOTION: BasicMotionStageValidator(),
            TaskKind.VISUAL_GOAL_NAVIGATION: VisualGoalStageValidator(
                limits, settings.completion_rules
            ),
        }
        terminal_kinds: set[TaskKind] = set()
        closed = [basic, visual]
        if free_planner is not None:
            free = FreeNavigationWorkflow(
                vehicle,
                free_planner,
                validator,
                motion,
                execution,
                model_request_timeout_s=config.trajectory_planner.timeout_s,
                sliding_window=config.sliding_window,
                motion_limits=vehicle.tracking_limits,
                moving_observation_source=(
                    BorrowedMovingSource(vehicle)
                    if config.sliding_window.enabled
                    else None
                ),
                stream_speed_mps=config.motion.max_speed_mps,
                stream_waypoint_timeout_s=60.0,
                collision_backoff_m=config.free_navigation.collision_backoff_m,
                maximum_collision_recoveries=(
                    config.free_navigation.maximum_collision_recoveries
                ),
                collision_recovery_reset_m=(
                    config.free_navigation.collision_recovery_reset_m
                ),
                continuous_tracking=config.continuous,
                lane_reference_provider=None,
                telemetry=self._trajectory,
            )
            workflows[TaskKind.FREE_NAVIGATION] = free
            stage_validators[TaskKind.FREE_NAVIGATION] = (
                FreeNavigationStageValidator()
            )
            terminal_kinds.add(TaskKind.FREE_NAVIGATION)
            closed.append(free)
        router = WorkflowRouter(workflows)
        mission_validator = MissionValidator(
            stage_validators, terminal_kinds=terminal_kinds
        )
        parser = build_task_parser(
            config,
            kinds=router.supported_kinds,
            aerial=vehicle_id == "drone",
        )
        model_calls = None
        application = None
        if parser is None:
            self._log(
                "task parser is not configured; tasks will be rejected",
                "WARNING",
            )
        else:
            model_calls = ModelCallService()
            parser_timeout_s = config.task_parser.timeout_s

            def parse_call(instruction: str):
                return model_calls.call(
                    lambda: parser.parse(instruction),
                    parser_timeout_s,
                    execution.checkpoint,
                )

            application = PlannerApplication(
                parser,
                mission_validator,
                MissionRunner(router),
                parse_call=parse_call,
            )
        return _VehicleArm(
            vehicle_id,
            vehicle,
            vehicle is self._drone,
            execution,
            application,
            model_calls,
            tuple(closed),
        )

    # -- ports -------------------------------------------------------------
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
        if arm.application is None or arm.execution is None:
            raise ProtocolError(
                "task parser is not configured; set task_parser.url/model "
                "in the configuration (strict parsing has no fallback)"
            )
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
            plan = arm.application.prepare(task.instruction)
            event_sink(
                f"{vehicle_id} mission plan: "
                + ", ".join(
                    f"{stage.stage_id}:{stage.kind.value}"
                    for stage in plan.stages
                )
            )
            self._prepare_vehicle(arm, event_sink)
            return arm.application.execute_plan(plan)
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
        """Advance the paused world one control period on the owner thread."""
        if self._active_vehicle == "drone" and self._drone is not None:
            self._drone.idle_tick()
        elif self._rover is not None:
            self._rover.idle_tick()
        elif self._connection is not None:
            self._connection.advance_for_sim_time(DEFAULT_CONTROL_PERIOD_S)

    def return_home(self, vehicle: str | None = None) -> str:
        """Teleport the requested vehicle to its measured spawn pose."""
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
        return vehicle_id

    def activate_vehicle(self, vehicle: str | None = None) -> str:
        """Console switch: park the other vehicle and home + show this one.

        This is the one atomic operation behind clicking a vehicle in the
        console. Parking is intentionally not modelled as homing the other
        vehicle: the drone is landed where it is, the rover is braked where it
        is, then the other vehicle is teleported out of view so only one
        vehicle is visible. The requested vehicle is teleported to its spawn.
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
        try:
            # The requested vehicle is usually still parked above the map
            # (hidden when the other vehicle was active). Restore it before
            # homing: the altitude pin would otherwise drag it back on every
            # clock advance and the drone's home convergence check fails at
            # about 1000 m, landing the vehicle far above the map.
            self._unhide(vehicle_id)
        except Exception as error:
            raise ExecutionError(
                f"could not restore the {vehicle_id}; "
                f"{vehicle_id} was not activated"
            ) from error
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

    @property
    def active_vehicle(self) -> str:
        return self._active_vehicle

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
        values = validate_weather_values(fields)
        if self._connection is None or self._connection.world is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        self._connection.reset_weather_effects()
        if all(value == 0.0 for value in values.values()):
            if not self._connection.disable_weather_effects():
                raise ProtocolError("simulator refused to disable weather effects")
            self._weather = values
            self._log("Weather cleared; all visual effects disabled.")
            return dict(values)
        if not self._connection.enable_weather_effects():
            raise ProtocolError("simulator refused to enable weather effects")
        for name in WEATHER_FIELDS:
            value = values[name]
            if value == 0.0:
                continue
            parameter = int(getattr(WeatherParameter, name.upper()))
            if not self._connection.set_weather_parameter(parameter, value):
                raise ProtocolError(
                    f"simulator refused weather.{name}={value:.2f}"
                )
        self._weather = values
        self._log(
            "Weather applied: "
            + ", ".join(
                f"{name}={values[name]:.2f}"
                for name in WEATHER_FIELDS
                if values[name] > 0.0
            )
        )
        return dict(values)

    def set_view_mode(self, mode: str) -> str:
        """Cycle the streaming view forward to the requested camera."""
        mode = validate_view_mode(mode)
        target = self._view_positions.index(mode)
        steps = cycle_delta(self._view_index, target)
        if self._connection is None or self._connection.world is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        for _ in range(steps):
            if not self._connection.switch_streaming_view():
                raise ProtocolError(
                    "simulator refused to switch the streaming view"
                )
        self._view_index = target
        self._view_mode = self._view_positions[target]
        self._log(f"Streaming view: {self._view_mode}.")
        return self._view_mode

    # -- internals ---------------------------------------------------------
    def _prepare_vehicle(self, arm: _VehicleArm, log) -> None:
        if arm.is_drone:
            self._drone.prepare(takeoff_if_landed=True)
            log("Drone airborne; position hold confirmed.")
        else:
            # start() already armed and braked the rover.
            self._rover.hover()

    def _vehicle_for(self, vehicle_id: str):
        vehicle = self._drone if vehicle_id == "drone" else self._rover
        if vehicle is None:
            raise RuntimeError("the ProjectAirSim session is closed")
        return vehicle

    def _calibrate_views(self) -> None:
        """Measure the streaming-view cycle order against the live scene.

        The server cycles through the streaming cameras in an order that is
        not stable across builds or scene loads, so the static ``VIEW_MODES``
        order cannot be trusted for the console views. Calibration walks one
        full cycle, matches xpra screenshots against camera frames fetched
        with ``GetImages`` and stores the measured order. Any failure keeps
        the built-in order and only logs a warning.

        The measured order is cached per scene file; a later session reuses it
        after one screenshot confirms which camera the cycle is currently on.
        """
        if self._connection is None or self._connection.world is None:
            return
        settings = self._config.projectairsim
        parent_topic = getattr(self._connection.world, "parent_topic", None)
        if not parent_topic:
            self._log(
                "streaming-view calibration skipped: no scene topic",
                "WARNING",
            )
            self._apply_initial_view()
            return
        cameras = {
            "rover_front": (settings.rover_name, settings.front_camera),
            "rover_chase": (settings.rover_name, settings.chase_camera),
            "drone_front": (settings.drone_name, settings.front_camera),
            "drone_chase": (settings.drone_name, settings.chase_camera),
        }
        display = self._config.simulator.display
        uncapturable = tuple(
            getattr(settings, "uncapturable_views", ())
        )
        fetch_frame = service_frame_fetcher(
            address=settings.address,
            port_topics=settings.port_topics,
            port_services=settings.port_services,
            parent_topic=parent_topic,
            advance=self._connection.advance_for_sim_time,
        )
        try:
            if self._reuse_cached_views(
                cameras, display, uncapturable, fetch_frame
            ):
                return
            positions = calibrate_streaming_views(
                cameras=cameras,
                screenshot=lambda: xpra_screenshot(display),
                fetch_frame=fetch_frame,
                switch_view=self._connection.switch_streaming_view,
                advance=self._connection.advance_for_sim_time,
                uncapturable=uncapturable,
                log=self._log,
            )
        except Exception as error:  # noqa: BLE001
            self._log(
                "streaming-view calibration failed: "
                f"{type(error).__name__}: {error}",
                "WARNING",
            )
            self._apply_initial_view()
            return
        finally:
            close = getattr(fetch_frame, "close", None)
            if close is not None:
                close()
        if positions is None:
            self._log(
                "streaming-view calibration could not identify the cycle; "
                "falling back to the built-in view order",
                "WARNING",
            )
            self._apply_initial_view()
            return
        self._view_positions = positions
        self._view_index = 0
        self._view_mode = positions[0]
        self._cache_view_order(positions)
        self._log(
            "Streaming views calibrated: "
            + " -> ".join(
                f"{index}:{mode}" for index, mode in enumerate(positions)
            )
        )
        # Start on the default vehicle's chase (tail) camera so the console's
        # initial view state matches what the simulator is actually showing.
        self._apply_initial_view()

    def _reuse_cached_views(
        self, cameras, display: str, uncapturable, fetch_frame
    ) -> bool:
        """Reuse the last measured cycle order after a cheap anchor check.

        The cycle is a ring, so one confidently identified camera at the
        current position is enough to index the cached order; a screenshot
        that is not a confident match (usually the uncapturable chase camera)
        falls back to the full calibration.
        """
        cache = getattr(self, "_view_cache", None)
        if not cache or self._connection is None:
            return False
        key = self._config.projectairsim.scene_file
        expected = cache.get(key)
        if expected is None:
            return False
        anchor = verify_streaming_views(
            cameras=cameras,
            screenshot=lambda: xpra_screenshot(display),
            fetch_frame=fetch_frame,
            switch_view=self._connection.switch_streaming_view,
            advance=self._connection.advance_for_sim_time,
            expected=expected,
            uncapturable=uncapturable,
            log=self._log,
        )
        if anchor is None:
            self._log(
                "Cached streaming-view order did not verify; calibrating again.",
                "WARNING",
            )
            return False
        positions = tuple(expected)
        self._view_positions = positions
        self._view_index = positions.index(anchor)
        self._view_mode = anchor
        self._log(
            "Streaming views reused from cache (current "
            f"{anchor}): "
            + " -> ".join(
                f"{index}:{mode}" for index, mode in enumerate(positions)
            )
        )
        self._apply_initial_view()
        return True

    def _cache_view_order(self, positions: tuple[str, ...]) -> None:
        cache = getattr(self, "_view_cache", None)
        if cache is not None:
            cache[self._config.projectairsim.scene_file] = tuple(positions)

    def _apply_initial_view(self) -> None:
        """Park the streaming view on the active vehicle's chase (tail) camera.

        Runs on both calibration outcomes: a failed calibration must not leave
        the console on whatever camera the calibration loop stopped at.
        """
        try:
            self.set_view_mode(default_view_for(self._active_vehicle))
        except Exception as error:  # noqa: BLE001
            self._log(
                "could not switch to the initial streaming view: "
                f"{type(error).__name__}: {error}",
                "WARNING",
            )

    def _hide_inactive_at_startup(self) -> None:
        """Move the non-default vehicle out of view right after loading."""
        other = "drone" if self._active_vehicle == "rover" else "rover"
        try:
            self._hide(other)
        except Exception as error:
            self._log(
                f"could not move the {other} out of view: "
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
        ``_pin_hidden`` keeps it at altitude while the world is stepped.
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
            except Exception as error:  # noqa: BLE001
                if vehicle_id not in self._pin_failures:
                    self._pin_failures.add(vehicle_id)
                    self._log(
                        f"could not keep the hidden {vehicle_id} at altitude: "
                        f"{type(error).__name__}: {error}",
                        "WARNING",
                    )
            else:
                self._pin_failures.discard(vehicle_id)

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

    def _park(self, vehicle_id: str, log) -> bool:
        """Put an inactive vehicle into a safe, motionless state.

        Returns False only when a landing was required but not confirmed.
        """
        try:
            if vehicle_id == "drone" and self._drone is not None:
                if not self._drone.landed:
                    self._drone.cancel()
                    if not self._drone.land():
                        log("Drone landing was not confirmed before switching.")
                        return False
                log("Drone parked on the ground.")
            elif self._rover is not None:
                self._rover.hover()
            return True
        except Exception as error:
            log(
                f"parking the {vehicle_id} failed: "
                f"{type(error).__name__}: {error}"
            )
            return False

    def _settle(self, arm: _VehicleArm, log) -> None:
        """Keep the reference post-task hold; land the drone if it fails."""
        try:
            if arm.is_drone:
                self._drone.hold(require_airborne=False)
            else:
                self._rover.hover()
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

    def _sink(self, frame: np.ndarray, timestamp_s: float) -> None:
        if self._frame_sink is not None and self._publish_frames:
            self._frame_sink(frame, timestamp_s)

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
                except Exception as error:  # noqa: BLE001
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