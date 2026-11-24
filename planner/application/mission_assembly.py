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

from ..capabilities.approach import TargetApproach
from ..capabilities.completion.visual_goal import VisualGoalCompletion
from ..capabilities.targeting.bbox_goal_observation import BBoxGoalObserver
from ..capabilities.task_validation.basic_motion import BasicMotionStageValidator
from ..capabilities.task_validation.free_navigation import FreeNavigationStageValidator
from ..capabilities.task_validation.visual_goal_navigation import (
    VisualGoalStageValidator,
)
from ..domain.mission import TaskKind
from ..errors import ProtocolError
from ..ports.path_vehicle import PathVehicle
from ..services.execution import ExecutionService
from ..services.model_calls import ModelCallService
from ..services.motion import SynchronizedMotionPlanner
from ..services.trajectory_validation import TrajectoryValidator
from ..simulator.ports import PIPELINE_REQUIREMENTS, SimulatorPorts
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
    enabled_tasks: tuple[str, ...] | None = None
    #: The engine path-following port, when the session implements one. ``None``
    #: means this simulator cannot fly the bare pipeline, and the assembly says so
    #: instead of falling back to stopped segments.
    path_follow: PathVehicle | None = None


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
            enabled_tasks=inputs.enabled_tasks,
            execution=inputs.execution,
            observation_source=inputs.observation_source,
            path_follow=inputs.path_follow,
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


@dataclass(frozen=True)
class VisualPipelineContext:
    """What one vehicle arm can hand a visual-goal pipeline.

    The two pipelines read different subsets: staged flies stopped segments and
    needs the validator, the motion planner and the moving source; bare hands the
    batch to the engine and needs ``path_follow``. One record keeps every builder
    in the table callable by the same expression.
    """

    config: object
    settings: object
    common: MissionCommon
    vehicle: object
    source: object
    goal_observer: object
    planner: object
    geometry: object
    execution: ExecutionService
    validator: TrajectoryValidator
    motion: SynchronizedMotionPlanner
    moving_observation_source: object | None
    path_follow: PathVehicle | None
    telemetry: object | None
    is_drone: bool


def _staged_visual_workflow(context: VisualPipelineContext) -> object:
    """The world-model pipeline: stopped segments, smoothed and validated."""
    config = context.config
    settings = context.settings
    limits = settings.limits
    return VisualGoalNavigationWorkflow(
        context.source,
        context.goal_observer,
        context.planner,
        context.validator,
        context.motion,
        context.execution,
        context.geometry,
        VisualGoalCompletion(limits, TargetApproach()),
        limits,
        trace_factory=context.common.trace_factory,
        completion_checker=context.common.completion_checker,
        trigger_rules=settings.completion_rules,
        sliding_window=config.sliding_window,
        motion_limits=context.vehicle.tracking_limits,
        collision_check=(
            context.vehicle.collision_state
            if settings.collision_check_enabled
            else None
        ),
        moving_observation_source=context.moving_observation_source,
        stream_speed_mps=config.motion.max_speed_mps,
        stream_waypoint_timeout_s=60.0,
        continuous_tracking=config.continuous,
        telemetry=context.telemetry,
        vertical_motion=context.is_drone,
    )


def _bare_visual_workflow(context: VisualPipelineContext) -> object:
    """The bare pipeline: one instruction, one batch, the engine follows it."""
    # Imported where it is built so that a staged arm does not depend on the bare
    # workflow's own imports.
    from ..workflows.visual_goal_navigation.bare import BareVisualGoalNavigation

    settings = context.settings
    return BareVisualGoalNavigation(
        context.source,
        context.goal_observer,
        context.planner,
        context.path_follow,
        context.geometry,
        trigger_rules=settings.completion_rules,
        limits=settings.limits,
        bare=settings.bare,
        telemetry=context.telemetry,
        vertical_motion=context.is_drone,
    )


#: Visual-goal pipeline name -> builder. ``execution`` indexes this table, and an
#: unknown name is a construction error rather than a quiet fall back to staged.
#: A new pipeline needs one row here and one in
#: ``planner.simulator.ports.PIPELINE_REQUIREMENTS``.
VISUAL_PIPELINES: dict[str, Callable[[VisualPipelineContext], object]] = {
    "staged": _staged_visual_workflow,
    "bare": _bare_visual_workflow,
}


def visual_pipeline(execution: str) -> Callable[[VisualPipelineContext], object]:
    builder = VISUAL_PIPELINES.get(execution)
    if builder is None:
        raise ProtocolError(
            f"unknown visual-goal execution {execution!r}; expected "
            f"{sorted(VISUAL_PIPELINES)}"
        )
    return builder


#: Slot -> the callable a pipeline makes on it. A slot filled with the wrong
#: object passes ``missing`` but cannot fly, which is a wiring mistake and is
#: reported with the slot it came from.
SLOT_METHODS: dict[str, str] = {"path_follow": "follow_path"}


def require_pipeline_ports(ports: SimulatorPorts, execution: str) -> None:
    """Refuse to build a pipeline this simulator cannot fly."""
    required = PIPELINE_REQUIREMENTS.get(execution)
    if required is None:
        raise ProtocolError(
            f"unknown visual-goal pipeline {execution!r}; expected "
            f"{sorted(PIPELINE_REQUIREMENTS)}"
        )
    missing = ports.missing(required)
    if missing:
        raise ProtocolError(
            f"visual-goal pipeline {execution!r} needs port slots this simulator "
            f"does not implement: {list(missing)}"
        )
    for slot in required:
        method = SLOT_METHODS.get(slot)
        if method is None:
            continue
        held = ports.slot(slot)
        if not callable(getattr(held, method, None)):
            raise ProtocolError(
                f"visual-goal pipeline {execution!r} needs port slot {slot!r} to "
                f"provide {method}(); got {type(held).__name__}"
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
    enabled_tasks: tuple[str, ...] | None = None,
    execution: ExecutionService | None = None,
    observation_source=None,
    path_follow: PathVehicle | None = None,
    log: Callable[[str, str], None],
) -> VehicleArm:
    """Wire the task pipeline against one vehicle port.

    ``observation_source`` is the object the visual-goal loop reads pixels from. It
    defaults to ``vehicle`` because the ProjectAirSim vehicles also capture, but the
    two are different ports: AirSim keeps capture in its own source with a second RPC
    connection, and passing the vehicle there made the stage die on
    "'AirSimVehicle' object has no attribute 'capture'" the moment it looked at an
    image. Say which object plays which role instead of hoping one plays both.

    ``enabled_tasks`` is the simulator package's own list (``ENABLED_TASKS``). A
    family that has never flown a task must not be offered it because a model
    endpoint happens to be configured: UE4 has no free-navigation history with
    main's planner, and the task parser prompt is built from the kinds the router
    actually supports, so filtering here also filters the prompt.
    """
    settings = config.visual_goal_navigation
    limits = settings.limits
    is_drone = vehicle_id == "drone"
    allowed = None if enabled_tasks is None else set(enabled_tasks)
    planner = common.aerial_planner if is_drone else common.planner
    free_planner = common.aerial_free_planner if is_drone else common.free_planner
    # AirSim passes an execution service it has already bound to the position hold and
    # the observation gate; every other caller gets a plain one.
    execution = ExecutionService(vehicle) if execution is None else execution
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
    # Only the staged pipeline reads the moving source; the bare one has no such
    # argument, so a bare arm must not pay for a capture source nothing reads.
    moving_source = (
        moving_source_factory(vehicle)
        if settings.execution == "staged"
        and moving_source_factory is not None
        and config.sliding_window.enabled
        else None
    )
    pipeline = visual_pipeline(settings.execution)
    # The bundle the slots are read from, built from the objects this arm already has:
    # a pipeline that needs a port this simulator leaves empty must fail here, not
    # mid-flight. Staged needs the vehicle and the image, bare also needs the path.
    require_pipeline_ports(
        SimulatorPorts(vehicle=vehicle, observation=source, path_follow=path_follow),
        settings.execution,
    )
    visual = pipeline(
        VisualPipelineContext(
            config=config,
            settings=settings,
            common=common,
            vehicle=vehicle,
            source=source,
            goal_observer=goal_observer,
            planner=planner,
            geometry=geometry,
            execution=execution,
            validator=validator,
            motion=motion,
            moving_observation_source=moving_source,
            path_follow=path_follow,
            telemetry=telemetry,
            is_drone=is_drone,
        )
    )
    basic = BasicMotionWorkflow(
        vehicle,
        motion,
        execution,
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
    if allowed is not None:
        workflows = {kind: workflow for kind, workflow in workflows.items()
                     if kind.value in allowed}
        stage_validators = {kind: validator for kind, validator in stage_validators.items()
                            if kind.value in allowed}
        closed = list(workflows.values())
    if free_planner is not None and (allowed is None or "free_navigation" in allowed):
        free = FreeNavigationWorkflow(
            vehicle,
            free_planner,
            validator,
            motion,
            execution,
            model_request_timeout_s=config.trajectory_planner.timeout_s,
            sliding_window=config.sliding_window,
            motion_limits=vehicle.tracking_limits,
            moving_observation_source=moving_source,
            stream_speed_mps=config.motion.max_speed_mps,
            stream_waypoint_timeout_s=60.0,
            collision_backoff_m=config.free_navigation.collision_backoff_m,
            maximum_collision_recoveries=(
                config.free_navigation.maximum_collision_recoveries
            ),
            collision_recovery_reset_m=config.free_navigation.collision_recovery_reset_m,
            continuous_tracking=config.continuous,
            lane_reference_provider=None,
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
    )


def require_application(arm: VehicleArm) -> PlannerApplication:
    """Fail with the operator's fix instead of running a task against nothing."""
    if arm.application is None or arm.execution is None:
        raise ProtocolError(
            "task parser is not configured; set task_parser.url/model "
            "in the configuration (strict parsing has no fallback)"
        )
    return arm.application
