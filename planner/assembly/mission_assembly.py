"""One assembly for the mission application, whatever simulator owns the vehicle.

The task pipeline -- trajectory planner, validators, workflows, stage validation,
task parser and ``PlannerApplication`` -- is simulator-independent: it is wired to
the ``Vehicle``/``ObservationSource`` ports. It used to be written inside
``ProjectAirSimMissionSession``, which meant that flying UE4 required either
importing a UE5 module that loads ``projectairsim`` in the worker process, or
copying ~170 lines. The copy is the mistake this repository already paid for once
(12044 lines in ``planner/ue4/``), so the wiring lives here and both families call it.

Nothing in this module imports a simulator client library; ``tests/test_architecture.py``
holds that line.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from planner.workflows.visual_goal_navigation import TargetApproach
from planner.workflows.visual_goal_navigation import VisualGoalCompletion
from planner.workflows.visual_goal_navigation import BBoxGoalObserver
from planner.workflows.basic_motion import BasicMotionStageValidator
from planner.workflows.free_navigation import FreeNavigationStageValidator
from planner.workflows.visual_goal_navigation import VisualGoalStageValidator
from planner.domain import TaskKind
from ..errors import ProtocolError
from planner.runtime import ControlLoop
from planner.runtime import ExecutionService
from planner.runtime import GuidedVehicle
from planner.runtime import ModelCallService
from planner.runtime import SynchronizedMotionPlanner
from planner.algorithms import TrajectoryValidator
from ..workflows.basic_motion import BasicMotionWorkflow
from ..workflows.free_navigation import FreeNavigationWorkflow
from ..workflows.visual_goal_navigation import VisualGoalNavigationWorkflow
from .application import PlannerApplication
from .factories import (
    build_completion_checker,
    build_free_navigation_planner,
    build_planner,
    build_task_parser,
    build_trace_factory,
)
from .mission_runner import MissionRunner
from .mission_validator import MissionValidator
from .workflow_router import WorkflowRouter


@dataclass(frozen=True)
class MissionResources:
    """What every session needs that does not depend on the simulator.

    The HTTP bootstrap builds these once and hands the same objects to every
    session it creates. Passing them as one record keeps ``registry.build_session``
    from growing five more keyword arguments each time a service appears.
    """

    detector: object
    planner: object
    geometry: object
    trajectory: object | None = None
    frame_sink: Callable | None = None


@dataclass(frozen=True)
class MissionInputs:
    """The engine-side objects the assembly wires a task pipeline around.

    A simulator session owns these and hands them over. It does not get to build
    planners, detectors or telemetry sinks: two peer middlewares may not
    construct each other, and this assembly is the only place allowed to know
    both. That inversion is what made the console's trajectory panel a UE4-only
    mystery -- the child process built its own resources, forgot one sink, and
    the workflow published nothing without saying so.
    """

    #: vehicle id -> the object that drives it
    vehicles: dict
    execution: object | None = None
    observation_source: object | None = None
    moving_source_factory: Callable | None = None


def assemble_missions(
    config, *, inputs: MissionInputs, resources, log
) -> dict:
    """Wire every vehicle the simulator exposes into a task pipeline.

    Returns ``{vehicle_id: VehicleArm}``. ``resources.trajectory`` is required:
    it used to be an optional argument the visual-goal workflow answered by
    publishing nothing, so a family that failed to pass it lost the console's
    remaining-trajectory panel silently. A missing sink is now a construction
    error, which is the only kind of error that reaches a developer.
    """
    if resources.trajectory is None:
        raise ProtocolError(
            "mission assembly requires a trajectory sink: the console reads "
            "the remaining path from it for every simulator"
        )
    common = build_mission_common(
        config,
        planner=resources.planner,
        detector=resources.detector,
        log=log,
    )
    return {
        vehicle_id: build_vehicle_arm(
            config,
            common=common,
            vehicle=vehicle,
            vehicle_id=vehicle_id,
            detector=resources.detector,
            geometry=resources.geometry,
            telemetry=resources.trajectory,
            moving_source_factory=inputs.moving_source_factory,
            execution=inputs.execution,
            observation_source=inputs.observation_source,
            log=log,
        )
        for vehicle_id, vehicle in inputs.vehicles.items()
    }


@dataclass(frozen=True)
class MissionCommon:
    """The per-service objects every vehicle arm shares."""

    planner: object
    aerial_planner: object
    free_planner: object | None
    aerial_free_planner: object | None
    completion_checker: object
    trace_factory: object


@dataclass
class VehicleArm:
    """One vehicle plus the application that can fly or drive it."""

    vehicle_id: str
    vehicle: object
    is_drone: bool
    execution: ExecutionService
    application: PlannerApplication | None
    model_calls: ModelCallService | None
    workflows: tuple
    loop: ControlLoop | None = None


def build_mission_common(config, *, planner, detector, log) -> MissionCommon:
    """Build what does not depend on which vehicle is asking.

    The drone keeps the reference planner's UAV prompts and the rover keeps
    carla-vln's ground-vehicle prompts. Geometric mode has no prompt, so the aerial
    variant reuses the already-built planner.
    """
    free_planner = build_free_navigation_planner(config)
    aerial_free_planner = build_free_navigation_planner(config, aerial=True)
    aerial_planner = (
        build_planner(config, config.server.planner, aerial=True)
        if config.server.planner == "vlm"
        else planner
    )
    if free_planner is None:
        log(
            "free navigation planner is not configured; "
            "free_navigation stages will be rejected",
            "WARNING",
        )
    return MissionCommon(
        planner=planner,
        aerial_planner=aerial_planner,
        free_planner=free_planner,
        aerial_free_planner=aerial_free_planner,
        completion_checker=build_completion_checker(config),
        trace_factory=build_trace_factory(config, detector=detector, planner=planner),
    )


def build_vehicle_arm(
    config,
    *,
    common: MissionCommon,
    vehicle,
    vehicle_id: str,
    detector,
    geometry,
    telemetry=None,
    moving_source_factory: Callable | None = None,
    execution: ExecutionService | None = None,
    observation_source=None,
    log: Callable[[str, str], None],
) -> VehicleArm:
    """Wire the task pipeline against one vehicle port.

    ``observation_source`` is the object the visual-goal loop reads pixels from. It
    defaults to ``vehicle`` because the ProjectAirSim vehicles also capture, but the
    two are different ports: AirSim keeps capture in its own source with a second RPC
    connection, and passing the vehicle there made the stage die on
    "'AirSimVehicle' object has no attribute 'capture'" the moment it looked at an
    image. Say which object plays which role instead of hoping one plays both.

    Which tasks this arm offers is not stated here: the task parser prompt is built
    from the kinds the router actually supports, and the router holds every kind
    whose required ports this vehicle fills and whose planner is configured.
    """
    settings = config.visual_goal_navigation
    limits = settings.limits
    is_drone = vehicle_id == "drone"
    planner = common.aerial_planner if is_drone else common.planner
    free_planner = common.aerial_free_planner if is_drone else common.free_planner
    # AirSim passes an execution service it has already bound to the position hold and
    # the observation gate; every other caller gets a plain one.
    execution = ExecutionService(vehicle) if execution is None else execution
    loop = ControlLoop(
        execution,
        vehicle,
        limits=vehicle.tracking_limits,
        tracking=config.continuous,
        timeout_s=config.drone.move_timeout_s,
        collision=(
            config.collision.recovery() if config.collision.enabled else None
        ),
    )
    attach = getattr(vehicle, "attach_loop", None)
    if attach is not None:
        attach(loop)
    guided = GuidedVehicle(vehicle, loop)
    source = vehicle if observation_source is None else observation_source
    motion = SynchronizedMotionPlanner(config.motion.max_speed_mps, vehicle.tracking_limits)
    goal_observer = BBoxGoalObserver(
        detector,
        min(config.target_detection.timeout_s, limits.model_request_timeout_s),
    )
    validator = TrajectoryValidator(
        config.trajectory_planner.point_count,
        config.trajectory_planner.max_step_m,
        config.trajectory_planner.max_yaw_step_deg,
    )
    moving_source = (
        moving_source_factory(vehicle)
        if moving_source_factory is not None and config.sliding_window.enabled
        else None
    )
    visual = VisualGoalNavigationWorkflow(
        source,
        goal_observer,
        planner,
        validator,
        motion,
        execution,
        geometry,
        VisualGoalCompletion(limits, TargetApproach()),
        limits,
        loop=loop,
        trace_factory=common.trace_factory,
        completion_checker=common.completion_checker,
        trigger_rules=settings.completion_rules,
        sliding_window=config.sliding_window,
        motion_limits=vehicle.tracking_limits,
        moving_observation_source=moving_source,
        stream_speed_mps=config.motion.max_speed_mps,
        stream_waypoint_timeout_s=60.0,
        continuous_tracking=config.continuous,
        telemetry=telemetry,
        vertical_motion=is_drone,
    )
    basic = BasicMotionWorkflow(
        guided,
        motion,
        execution,
        loop=loop,
        limits=vehicle.tracking_limits,
        telemetry=telemetry,
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
            loop=loop,
            model_request_timeout_s=config.trajectory_planner.timeout_s,
            sliding_window=config.sliding_window,
            motion_limits=vehicle.tracking_limits,
            moving_observation_source=moving_source,
            stream_speed_mps=config.motion.max_speed_mps,
            stream_waypoint_timeout_s=60.0,
            continuous_tracking=config.continuous,
            telemetry=telemetry,
            vertical_motion=is_drone,
        )
        workflows[TaskKind.FREE_NAVIGATION] = free
        stage_validators[TaskKind.FREE_NAVIGATION] = FreeNavigationStageValidator()
        terminal_kinds.add(TaskKind.FREE_NAVIGATION)
        closed.append(free)
    router = WorkflowRouter(workflows)
    mission_validator = MissionValidator(stage_validators, terminal_kinds=terminal_kinds)
    parser = build_task_parser(config, kinds=router.supported_kinds, aerial=is_drone)
    application = None
    model_calls = None
    if parser is None:
        log(
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
    return VehicleArm(
        vehicle_id,
        vehicle,
        is_drone,
        execution,
        application,
        model_calls,
        tuple(closed),
        loop,
    )


def require_application(arm: VehicleArm) -> PlannerApplication:
    """Fail with the operator's fix instead of running a task against nothing."""
    if arm.application is None or arm.execution is None:
        raise ProtocolError(
            "task parser is not configured; set task_parser.url/model "
            "in the configuration (strict parsing has no fallback)"
        )
    return arm.application
