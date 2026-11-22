from types import SimpleNamespace

import numpy as np
import pytest

import planner.workflows.free_navigation.continuous as free_continuous_module
import planner.workflows.free_navigation.workflow as free_workflow_module
from planner.adapters.task_parser.task_factory import mission_from_dict
from planner.adapters.task_parser.tasks import builtin_task_codecs
from planner.adapters.task_parser.tasks.free_navigation import (
    decode_free_navigation,
)
from planner.adapters.trajectory_planner.prompts import (
    AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT,
    AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT,
    BASE_TRAJECTORY_SYSTEM_PROMPT,
    FREE_NAVIGATION_SYSTEM_PROMPT,
    FreeNavigationTrajectoryPrompt,
)
from planner.workflows.free_navigation import FreeNavigationStageValidator
from planner.domain import BasicMotionParameters
from planner.domain import ExecutionBudget, ExecutionRecord, StopResult
from planner.domain import FreeNavigationParameters, FreeNavigationStatus
from planner.domain import MissionStage, TaskKind
from planner.domain import MotionLimits
from planner.domain import CameraIntrinsics, Observation
from planner.domain import CollisionState, MotionCapability, MotionModel
from planner.domain import RelativePoseDelta, WorldPose
from planner.domain import QueuedWaypoint, SlidingWindowSettings
from planner.domain import QueueObservationAnchor
from planner.domain import RelativeTrajectory
from planner.domain import MODE_CONTINUOUS
from planner.errors import (
    ExecutionError,
    FreeNavigationStalled,
    ObservationError,
    ProtocolError,
)
from planner.workflows.free_navigation import FreeNavigationWorkflow
from planner.workflows.free_navigation import FreeNavigationContinuous

START = WorldPose(0.0, 0.0, 0.0, 0.0)


def _observation(
    *,
    stopped=True,
    pose=START,
    capture_id="initial",
    timestamp_ns=1,
    captured_monotonic=0.0,
):
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=pose,
        rgb=np.zeros((4, 4, 3), dtype=np.uint8),
        depth_m=np.ones((4, 4), dtype=np.float32),
        intrinsics=CameraIntrinsics.from_horizontal_fov(4, 4, 90.0),
        camera_matrix=np.eye(4),
        timestamp_ns=timestamp_ns,
        captured_monotonic=captured_monotonic,
        capture_id=capture_id,
        stopped=stopped,
    )


def _trajectory(dx=1.0):
    return RelativeTrajectory((RelativePoseDelta(dx, 0.0, 0.0, 0.0),))


def _stage():
    return MissionStage(
        "stage_1",
        TaskKind.FREE_NAVIGATION,
        FreeNavigationParameters("keep driving along the road"),
    )


class FakeSource:
    def __init__(self, observation, *, error=None):
        self.observation = observation
        self.error = error
        self.captures = 0

    def capture(self):
        self.captures += 1
        if self.error is not None:
            raise self.error
        return self.observation


class FakePlanner:
    def __init__(self, trajectory, *, fail_at=None, error=None):
        self.trajectory = trajectory
        self.fail_at = fail_at
        self.error = error
        self.calls = []

    def plan(self, observation, instruction, **kwargs):
        self.calls.append((observation, instruction, kwargs))
        if self.fail_at is not None and len(self.calls) == self.fail_at:
            raise self.error
        return self.trajectory


class FakeValidator:
    def __init__(self):
        self.relative = []
        self.world = []

    def validate_relative(self, trajectory):
        self.relative.append(trajectory)

    def validate_world(self, trajectory):
        self.world.append(trajectory)


class FakeMotion:
    def __init__(self):
        self.created = []

    def create_motion(self, world):
        self.created.append(world)
        return world


class FakeExecution:
    def __init__(self, *, record=None, execute_error=None):
        self.record = record
        self.execute_error = execute_error
        self.acquired = []
        self.released = []
        self.deadlines = []
        self.executed = []
        self.stops = 0
        self.checkpoints = 0

    def acquire_flight(self, owner):
        self.acquired.append(owner)

    def release_flight(self, owner, pending_cleanup=None):
        self.released.append((owner, pending_cleanup))

    def set_deadline(self, deadline):
        self.deadlines.append(deadline)

    def checkpoint(self):
        self.checkpoints += 1

    def stop_if_owner(self, owner):
        self.stops += 1
        return StopResult(False)

    def execute(self, motion):
        self.executed.append(motion)
        if self.execute_error is not None:
            raise self.execute_error
        return self.record


def _workflow(
    *,
    source=None,
    planner=None,
    validator=None,
    motion=None,
    execution=None,
    **kwargs,
):
    source = source or FakeSource(_observation())
    planner = planner or FakePlanner(_trajectory())
    validator = validator or FakeValidator()
    motion = motion or FakeMotion()
    execution = execution or FakeExecution(
        record=ExecutionRecord(START, START.offset(1.0, 0.0), _trajectory())
    )
    workflow = FreeNavigationWorkflow(
        source, planner, validator, motion, execution, loop=execution, **kwargs
    )
    return workflow, source, planner, validator, motion, execution


def test_parameters_require_a_non_empty_instruction():
    with pytest.raises(ValueError):
        FreeNavigationParameters("   ")
    assert FreeNavigationStatus.STUCK.value == "stuck"


def test_codec_is_registered_and_decodes_instruction_only():
    assert any(
        codec.kind is TaskKind.FREE_NAVIGATION
        for codec in builtin_task_codecs()
    )
    parameters = decode_free_navigation({"instruction": "follow the road"})
    assert isinstance(parameters, FreeNavigationParameters)


@pytest.mark.parametrize(
    "raw",
    [
        {},
        {"instruction": "  "},
        {"instruction": "follow the road", "forward_m": 5.0},
        {"instruction": "follow the road", "target_description": "a car"},
    ],
)
def test_codec_rejects_anything_but_a_non_empty_instruction(raw):
    with pytest.raises(ProtocolError):
        decode_free_navigation(raw)


def test_mission_factory_admits_a_free_navigation_stage():
    plan = mission_from_dict(
        {
            "protocol_version": "mission_plan_v1",
            "stages": [
                {
                    "stage_id": "stage_1",
                    "kind": "free_navigation",
                    "parameters": {"instruction": "follow the road"},
                }
            ],
        }
    )
    assert plan.stages[0].kind is TaskKind.FREE_NAVIGATION
    assert isinstance(plan.stages[0].parameters, FreeNavigationParameters)


def test_stage_validator_checks_kind_and_parameter_type():
    FreeNavigationStageValidator().validate(_stage())
    with pytest.raises(ProtocolError):
        FreeNavigationStageValidator().validate(
            MissionStage(
                "stage_1",
                TaskKind.FREE_NAVIGATION,
                BasicMotionParameters("drive", forward_m=1.0),
            )
        )
    with pytest.raises(ProtocolError):
        FreeNavigationStageValidator().validate(
            MissionStage(
                "stage_1",
                TaskKind.BASIC_MOTION,
                FreeNavigationParameters("follow the road"),
            )
        )


def test_system_prompt_appends_road_following_rules_to_the_base():
    prompt = FreeNavigationTrajectoryPrompt().system_prompt()
    assert prompt == FREE_NAVIGATION_SYSTEM_PROMPT
    assert prompt.startswith(BASE_TRAJECTORY_SYSTEM_PROMPT)
    assert "Free navigation road-following rules" in prompt
    assert "There is no arrival condition" in prompt
    assert "Never invent a destination" in prompt
    assert "Lane discipline rules" in prompt
    assert "not the widest open space" in prompt
    assert "Do not cross a lane boundary" in prompt
    assert "Keep right" in prompt
    assert "clear bearing window inside the current lane" in prompt
    assert "Mandatory aim check before output" in prompt
    assert "The total turn is a different quantity" in prompt


def test_aerial_system_prompt_keeps_road_rules_with_flight_wording():
    prompt = FreeNavigationTrajectoryPrompt(aerial=True).system_prompt()
    assert prompt == AERIAL_FREE_NAVIGATION_SYSTEM_PROMPT
    assert prompt.startswith(AERIAL_BASE_TRAJECTORY_SYSTEM_PROMPT)
    assert "autonomous UAV" in prompt
    assert "Keep the UAV centred above the current lane" in prompt
    assert "flight direction" in prompt
    assert "Free navigation road-following rules" in prompt
    assert "Lane discipline rules" in prompt
    assert "the car" not in prompt
    assert FreeNavigationTrajectoryPrompt().system_prompt() == (
        FREE_NAVIGATION_SYSTEM_PROMPT
    )
    assert "within about 5 deg of the cumulative lane yaw" in prompt
    assert "Weave counterexample" in prompt
    assert "left-right weave" in prompt
    assert "Lane-centring steering procedure" in prompt
    assert "Step-length default (open lane)" in prompt
    assert "Open-lane worked example" in prompt
    assert "never the default on an open lane" in prompt
    assert "tight-space exception" in prompt
    text = FreeNavigationTrajectoryPrompt(aerial=True).build_user_prompt(
        "follow the road", _observation(), 5
    )
    assert "Current UAV pose:" in text
    assert "NED down:" in text
    assert "Current vehicle pose:" not in text


def test_user_prompt_carries_the_instruction_and_rejects_task_context():
    prompt = FreeNavigationTrajectoryPrompt()
    text = prompt.build_user_prompt("follow the road", _observation(), 5)
    assert "follow the road" in text
    with pytest.raises(ProtocolError):
        prompt.build_user_prompt(
            "follow the road", _observation(), 5, task_context=object()
        )


def test_workflow_replans_with_the_previous_batch_and_validates_both_frames():
    trajectory = _trajectory()
    planner = FakePlanner(
        trajectory, fail_at=2, error=ProtocolError("planner stopped")
    )
    execution = FakeExecution(
        record=ExecutionRecord(
            START, START.offset(1.0, 0.0), trajectory
        )
    )
    workflow, _, _, validator, motion, _ = _workflow(
        planner=planner, execution=execution
    )

    result = workflow.run(_stage())

    assert result.success is False
    assert result.value.status is FreeNavigationStatus.MODEL_ERROR
    assert len(planner.calls) == 2
    first = planner.calls[0][2]
    assert first["round_index"] == 1
    assert first["previous_trajectory"] is None
    assert first["previous_trajectory_start"] is None
    assert first["initial_pose"] == START
    second = planner.calls[1][2]
    assert second["round_index"] == 2
    assert second["previous_trajectory"] is trajectory
    assert second["previous_trajectory_start"] == START
    assert validator.relative == [trajectory]
    assert len(validator.world) == 1
    assert len(motion.created) == 1
    assert execution.stops == 1


def test_workflow_acquires_flight_and_releases_pending_cleanup():
    planner = FakePlanner(
        _trajectory(), fail_at=1, error=ProtocolError("planner stopped")
    )
    workflow, _, _, _, _, execution = _workflow(planner=planner)

    workflow.run(_stage())

    assert execution.acquired == [workflow]
    assert execution.deadlines == [None]
    release_owner, cleanup = execution.released[0]
    assert release_owner is workflow
    assert callable(cleanup)
    assert cleanup() is False


def test_workflow_maps_an_execution_error():
    execution = FakeExecution(
        execute_error=ExecutionError("motion tracking failed")
    )
    workflow, _, _, _, _, _ = _workflow(execution=execution)

    result = workflow.run(_stage())

    assert result.success is False
    assert result.value.status is FreeNavigationStatus.EXECUTION_ERROR
    assert "motion tracking failed" in result.error
    assert execution.stops == 1


def test_workflow_stops_and_propagates_cancellation():
    source = FakeSource(_observation(), error=KeyboardInterrupt())
    workflow, _, _, _, _, execution = _workflow(source=source)

    with pytest.raises(KeyboardInterrupt):
        workflow.run(_stage())

    assert execution.stops == 1
    assert execution.released == [
        (workflow, workflow.has_pending_calls)
    ]


def test_workflow_reports_stuck_after_consecutive_zero_motion_batches():
    stalled = ExecutionRecord(START, START, RelativeTrajectory(()))
    execution = FakeExecution(record=stalled)
    planner = FakePlanner(_trajectory())
    workflow, _, _, _, _, _ = _workflow(
        planner=planner, execution=execution
    )

    result = workflow.run(_stage())

    assert result.value.status is FreeNavigationStatus.STUCK
    assert len(planner.calls) == 2
    assert len(execution.executed) == 2
    assert execution.stops == 1


def test_workflow_requires_a_stopped_observation():
    source = FakeSource(_observation(stopped=False))
    workflow, _, _, _, _, execution = _workflow(source=source)

    result = workflow.run(_stage())

    assert result.value.status is FreeNavigationStatus.OBSERVATION_ERROR
    assert "stopped observation" in result.error
    assert execution.stops == 1


def test_workflow_wraps_capture_failures_as_observation_errors():
    source = FakeSource(_observation(), error=RuntimeError("camera lost"))
    workflow, _, _, _, _, _ = _workflow(source=source)

    result = workflow.run(_stage())

    assert result.value.status is FreeNavigationStatus.OBSERVATION_ERROR
    assert "camera lost" in result.error


def test_prepare_rejects_a_closed_workflow():
    workflow, _, _, _, _, _ = _workflow()
    workflow.close()
    with pytest.raises(ExecutionError):
        workflow.prepare()


# --- Continuous (sliding-window) execution ---------------------------------


HOLONOMIC = MotionCapability(MotionModel.HOLONOMIC, 2.0, None, True)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeTimeline:
    def __init__(self, anchor_failures=1):
        self.anchor_failures = anchor_failures
        self.calls = []

    def at(self, timestamp_ns, stage_token):
        self.calls.append((timestamp_ns, stage_token))
        if self.anchor_failures:
            self.anchor_failures -= 1
            raise ProtocolError("no streaming sample for this frame")
        return QueueObservationAnchor(stage_token, 3, timestamp_ns, ())


class FakeStreamRecorder:
    def __init__(self):
        self.path_length_m = 0.0
        self.record = ExecutionRecord(START, START, RelativeTrajectory(()))
        self.timeline = FakeTimeline()
        self.initial = START
        self.actions = []
        self.begun = []

    def begin(self, queue, stage_token):
        self.begun.append((queue, stage_token))

    def sample(self, state):
        return state


class FakeQueueFollower:
    def __init__(self, *args, **kwargs):
        self.begun = []
        self.resets = 0

    def begin(self, queue):
        self.begun.append(queue)

    def reset(self):
        self.resets += 1

    def next(self, state, unwrapped_yaw_deg):
        return None


class FakeContinuousLoop:
    """一拍推进 ``advance_per_tick`` 的控制循环替身。"""

    def __init__(self, *, execution=None, stop_after=None):
        self.execution = execution
        self.limits = SimpleNamespace(control_hz=20.0)
        self.recorder = FakeStreamRecorder()
        self.state = SimpleNamespace(pose=START)
        self.advance_per_tick = 0.0
        self.ticks = 0
        self.stop_after = stop_after
        self.holding = True

    @property
    def period(self):
        return 1 / self.limits.control_hz

    def sample(self):
        self.recorder.sample(self.state)
        return self.state

    def stream_sink(self):
        return lambda command, duration_s: None

    def run_stream(self, goals, sink, *, deadline_monotonic=None):
        self.sample()
        self.ticks += 1
        if self.stop_after is not None and self.ticks > self.stop_after:
            raise KeyboardInterrupt
        self.recorder.path_length_m += self.advance_per_tick

    def execute(self, motion):
        return self.execution.execute(motion)


class FakeMovingFrames:
    def __init__(self, frames=(), *, delay=0):
        self.job = SimpleNamespace(future=None, busy=False)
        self.frames = list(frames)
        self.delay = delay
        self.begun = 0
        self.discarded = 0
        self.closed = 0
        self.requests = []

    def begin(self):
        self.begun += 1

    def poll(self):
        if self.delay:
            self.delay -= 1
            return None
        return self.frames.pop(0) if self.frames else None

    def request(self, budget):
        self.requests.append(budget)

    def discard(self):
        self.discarded += 1

    def close(self):
        self.closed += 1


class FakeQueue:
    def __init__(self, items=()):
        self.items = tuple(items)
        self.revision = 3
        self.anchors = []
        self.cleared = 0

    def require_anchor(self, measured, observed):
        self.anchors.append((measured, observed))

    def clear(self):
        self.cleared += 1
        self.items = ()
        self.revision += 1


class FakePlanningWindow:
    def __init__(self, *, due=True, items=()):
        self.queue = FakeQueue(items)
        self.background = SimpleNamespace(stage_token=11, discard=lambda: None)
        self.settings = SimpleNamespace(
            max_observation_age_s=30.0, max_buffer_points=10
        )
        self.record = None
        self.due = due
        self.submissions = []
        self.polls = 0

    def stream_due(self):
        return self.due

    def poll_stream(self, measured, checkpoint):
        self.polls += 1

    def submit_stream(
        self,
        observation,
        context,
        instruction,
        planning,
        anchor,
        on_request,
        checkpoint,
    ):
        self.submissions.append(
            (observation, context, instruction, planning, anchor)
        )
        on_request()
        return True


class FakeTelemetry:
    def __init__(self):
        self.published = []

    def publish_queue(self, **kwargs):
        self.published.append(kwargs)


class FakeStreamingExecution:
    def __init__(self, *, vehicle=None):
        self.vehicle = vehicle or SimpleNamespace(
            stream_state=lambda: None,
            stream_command=lambda *args: None,
            motion_capability=lambda: HOLONOMIC,
            collision_state=lambda: CollisionState(False),
            hover=lambda: None,
        )
        self.checkpoints = 0
        self.stops = 0
        self.executed = 0
        self.execute_motions = []
        self.allow_execute = False

    def checkpoint(self):
        self.checkpoints += 1

    def stop(self):
        self.stops += 1
        return StopResult(True)

    def execute(self, motion):
        if not self.allow_execute:
            raise AssertionError(
                "continuous mode must not call execution.execute"
            )
        self.executed += 1
        self.execute_motions.append(motion)


def _continuous_controller(
    monkeypatch,
    *,
    stream=None,
    frames=None,
    clock=None,
    stall_timeout_s=60.0,
    vehicle=None,
    telemetry=None,
    source=None,
    motion=None,
    maximum_collision_recoveries=3,
    collision_recovery_reset_m=20.0,
):
    frames = frames or FakeMovingFrames()
    clock = clock or FakeClock()
    execution = FakeStreamingExecution(vehicle=vehicle)
    stream = stream or FakeContinuousLoop(execution=execution)
    stream.execution = execution
    monkeypatch.setattr(
        free_continuous_module,
        "QueueFollower",
        lambda *args, **kwargs: FakeQueueFollower(),
    )
    monkeypatch.setattr(
        free_continuous_module,
        "MovingObservations",
        lambda *args, **kwargs: frames,
    )
    controller = FreeNavigationContinuous(
        execution,
        stream,
        source or object(),
        MotionLimits(),
        2.0,
        clock,
        clock.advance,
        motion=motion,
        maximum_collision_recoveries=maximum_collision_recoveries,
        collision_recovery_reset_m=collision_recovery_reset_m,
        model_request_timeout_s=5.0,
        stall_timeout_s=stall_timeout_s,
        telemetry=telemetry,
    )
    return controller, execution, stream, frames


def test_continuous_loop_submits_stopped_and_moving_frames(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousLoop(stop_after=2)
    moving = _observation(
        stopped=False, capture_id="moving", timestamp_ns=2
    )
    frames = FakeMovingFrames(frames=[moving], delay=1)
    controller, execution, stream, frames = _continuous_controller(
        monkeypatch, stream=stream, frames=frames, clock=clock
    )
    window = FakePlanningWindow()

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
        )

    assert [item[0].capture_id for item in window.submissions] == [
        "initial",
        "moving",
    ]
    assert window.queue.anchors == [(START, START)]
    assert window.submissions[1][4] == QueueObservationAnchor(
        window.background.stage_token, window.queue.revision, 2, ()
    )
    assert stream.recorder.begun == [
        (window.queue, window.background.stage_token)
    ]
    assert frames.begun == 1
    assert frames.discarded == 1
    assert frames.requests
    assert all(
        isinstance(budget, ExecutionBudget)
        and budget.remaining(clock()) > 0
        for budget in frames.requests
    )
    assert execution.executed == 0
    assert controller.planning_calls == 2


def test_continuous_collision_backs_off_clears_queue_and_replans(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousLoop(stop_after=2)
    recovered = _observation(
        stopped=True, capture_id="recovered", timestamp_ns=3
    )
    source = FakeSource(recovered)
    # The first read drains a contact latched before the stage started; only
    # the second read is a contact made while driving.
    collisions = iter([False, True, False, False, False, False])
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_command=lambda *args: None,
        motion_capability=lambda: HOLONOMIC,
        collision_state=lambda: CollisionState(next(collisions, False)),
        hover=lambda: None,
    )
    controller, execution, stream, frames = _continuous_controller(
        monkeypatch,
        stream=stream,
        clock=clock,
        vehicle=vehicle,
        source=source,
        motion=FakeMotion(),
    )
    execution.allow_execute = True
    window = FakePlanningWindow()

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
        )

    assert execution.stops == 1
    assert execution.executed == 1
    world = execution.execute_motions[0]
    assert world.start == START
    assert world.poses[0].x == pytest.approx(-3.0)
    assert window.queue.cleared == 1
    assert source.captures == 1
    assert controller.collision_recoveries == 1
    assert stream.recorder.initial == START
    assert stream.recorder.actions == []
    assert window.submissions
    assert window.submissions[0][0].capture_id == "recovered"
    assert frames.discarded >= 1


def test_continuous_discards_a_contact_latched_before_the_stage(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousLoop(stop_after=2)
    collisions = iter([True, False, False])
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_command=lambda *args: None,
        motion_capability=lambda: HOLONOMIC,
        collision_state=lambda: CollisionState(next(collisions, False)),
        hover=lambda: None,
        last_collision=SimpleNamespace(object_name="Rover1"),
    )
    controller, execution, stream, _frames = _continuous_controller(
        monkeypatch,
        stream=stream,
        clock=clock,
        vehicle=vehicle,
        motion=FakeMotion(),
    )
    execution.allow_execute = True

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            FakePlanningWindow(),
        )

    assert controller.collision_recoveries == 0
    assert execution.executed == 0
    assert execution.stops == 0


def test_continuous_collision_recovery_budget_exhausts(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousLoop(stop_after=10)
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_command=lambda *args: None,
        motion_capability=lambda: HOLONOMIC,
        collision_state=lambda: CollisionState(True),
        hover=lambda: None,
    )
    controller, execution, stream, _frames = _continuous_controller(
        monkeypatch,
        stream=stream,
        clock=clock,
        vehicle=vehicle,
        source=FakeSource(_observation(capture_id="recovered")),
        motion=FakeMotion(),
        maximum_collision_recoveries=1,
    )
    execution.allow_execute = True

    with pytest.raises(ExecutionError, match="recovery budget exhausted"):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            FakePlanningWindow(),
        )

    assert controller.collision_recoveries == 2
    assert execution.executed == 1


def test_continuous_collision_backoff_escalates_per_recovery(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousLoop()
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_command=lambda *args: None,
        motion_capability=lambda: HOLONOMIC,
        collision_state=lambda: CollisionState(True),
        hover=lambda: None,
    )
    controller, execution, stream, _frames = _continuous_controller(
        monkeypatch,
        stream=stream,
        clock=clock,
        vehicle=vehicle,
        source=FakeSource(_observation(capture_id="recovered")),
        motion=FakeMotion(),
        maximum_collision_recoveries=3,
    )
    execution.allow_execute = True

    with pytest.raises(ExecutionError, match="recovery budget exhausted"):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            FakePlanningWindow(),
        )

    assert controller.collision_recoveries == 4
    assert execution.executed == 3
    assert [motion.poses[0].x for motion in execution.execute_motions] == [
        -3.0,
        -6.0,
        -9.0,
    ]


def test_continuous_collision_budget_resets_after_progress(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousLoop(stop_after=3)
    stream.advance_per_tick = 2.0
    # Each recovery reads collision_state once more to clear the latch; the
    # leading False is the pre-task drain read.
    collisions = iter([False, True, False, False, True, False])
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_command=lambda *args: None,
        motion_capability=lambda: HOLONOMIC,
        collision_state=lambda: CollisionState(next(collisions, False)),
        hover=lambda: None,
    )
    controller, execution, stream, _frames = _continuous_controller(
        monkeypatch,
        stream=stream,
        clock=clock,
        vehicle=vehicle,
        source=FakeSource(_observation(capture_id="recovered")),
        motion=FakeMotion(),
        maximum_collision_recoveries=1,
        collision_recovery_reset_m=1.0,
    )
    execution.allow_execute = True

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            FakePlanningWindow(),
        )

    assert controller.collision_recoveries == 0
    assert execution.executed == 2


def test_continuous_publishes_remaining_waypoint_telemetry(monkeypatch):
    telemetry = FakeTelemetry()
    stream = FakeContinuousLoop(stop_after=1)
    controller, _, _, _ = _continuous_controller(
        monkeypatch, stream=stream, telemetry=telemetry
    )
    item = QueuedWaypoint(
        9, START, START.offset(2.0, 0.0, 0.0, 15.0), 15.0
    )
    window = FakePlanningWindow(due=False, items=(item,))

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
            stage_id="stage_1",
        )

    assert telemetry.published
    last = telemetry.published[-1]
    assert last["instruction"] == "follow the road"
    assert last["stage_id"] == "stage_1"
    assert last["mode"] == MODE_CONTINUOUS
    assert last["capacity"] == 10
    assert last["items"] == (item,)
    assert last["pose"] == START


def test_continuous_watchdog_reports_a_stall_while_the_queue_is_not_draining(
    monkeypatch,
):
    stream = FakeContinuousLoop()
    controller, _, stream, frames = _continuous_controller(
        monkeypatch, stream=stream, stall_timeout_s=0.1
    )
    window = FakePlanningWindow(due=False, items=(object(),))

    with pytest.raises(FreeNavigationStalled):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
        )

    assert stream.ticks >= 1
    assert frames.discarded == 1
    assert controller.planning_calls == 0


def test_continuous_watchdog_does_not_stall_while_waiting_for_planning(
    monkeypatch,
):
    stream = FakeContinuousLoop(stop_after=6)
    controller, _, stream, _ = _continuous_controller(
        monkeypatch, stream=stream, stall_timeout_s=0.1
    )
    window = FakePlanningWindow(due=False, items=())

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
        )

    assert stream.ticks == 7


def test_continuous_prepare_requires_streaming_vehicle_and_rejects_closed(
    monkeypatch,
):
    controller, _, _, frames = _continuous_controller(monkeypatch)
    controller.prepare()
    frames.job.busy = True
    with pytest.raises(ExecutionError):
        controller.prepare()
    frames.job.busy = False
    controller.close()
    assert frames.closed == 1
    with pytest.raises(ExecutionError):
        controller.prepare()

    broken, _, _, _ = _continuous_controller(
        monkeypatch, vehicle=SimpleNamespace()
    )
    with pytest.raises(ExecutionError):
        broken.prepare()


@pytest.fixture
def continuous_stack(monkeypatch):
    windows, controllers = [], []

    class FakeWindow:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs
            self.worker = SimpleNamespace(
                require_available=lambda: None,
                has_pending_call=False,
            )
            self.begins, self.ends, self.closed = [], 0, 0
            windows.append(self)

        def begin(self, *, deadline_monotonic, maximum_planning_rounds):
            self.begins.append(
                (deadline_monotonic, maximum_planning_rounds)
            )

        def end(self):
            self.ends += 1

        def close(self):
            self.closed += 1

    class FakeController:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs
            self.prepared, self.closed = 0, 0
            self.planning_calls, self.path_length_m = 3, 12.0
            self.final_pose = START.offset(12.0, 0.0)
            self.error, self.pending, self.calls = None, False, []
            self.stage_ids = []
            controllers.append(self)

        def prepare(self):
            self.prepared += 1

        def has_pending_calls(self):
            return self.pending

        def close(self):
            self.closed += 1

        def run(self, parameters, observation, window, stage_id=None):
            self.calls.append((parameters, observation, window))
            self.stage_ids.append(stage_id)
            if self.error is not None:
                raise self.error

    monkeypatch.setattr(
        free_workflow_module, "PlanningWindow", FakeWindow
    )
    monkeypatch.setattr(
        free_workflow_module, "FreeNavigationContinuous", FakeController
    )
    return windows, controllers


def _continuous_workflow(continuous_stack, **kwargs):
    kwargs.setdefault("moving_observation_source", object())
    kwargs.setdefault(
        "sliding_window", SlidingWindowSettings(enabled=True)
    )
    kwargs.setdefault("motion_limits", MotionLimits())
    workflow, source, _planner, _validator, _motion, execution = _workflow(
        **kwargs
    )
    return workflow, source, execution


def test_continuous_mode_requires_an_enabled_window_and_motion_limits():
    def build(**kwargs):
        execution = FakeExecution()
        return FreeNavigationWorkflow(
            FakeSource(_observation()),
            FakePlanner(_trajectory()),
            FakeValidator(),
            FakeMotion(),
            execution,
            loop=execution,
            **kwargs,
        )

    with pytest.raises(ProtocolError):
        build(moving_observation_source=object())
    with pytest.raises(ProtocolError):
        build(
            moving_observation_source=object(),
            sliding_window=SlidingWindowSettings(enabled=False),
        )
    with pytest.raises(ValueError):
        build(
            moving_observation_source=object(),
            sliding_window=SlidingWindowSettings(enabled=True),
        )


def test_continuous_workflow_begins_unbounded_and_releases_pending_cleanup(
    continuous_stack,
):
    windows, controllers = continuous_stack
    workflow, source, execution = _continuous_workflow(continuous_stack)
    controllers[-1].error = KeyboardInterrupt()

    with pytest.raises(KeyboardInterrupt):
        workflow.run(_stage())

    window, controller = windows[-1], controllers[-1]
    assert controller.prepared == 1
    assert window.begins == [(None, None)]
    assert window.ends == 1
    assert execution.acquired == [workflow]
    assert execution.deadlines == [None]
    owner, cleanup = execution.released[0]
    assert owner is workflow
    assert callable(cleanup)
    assert cleanup() is False
    parameters, observation, opened = controller.calls[0]
    assert parameters.instruction == "keep driving along the road"
    assert observation.stopped is True
    assert opened is window
    assert controller.stage_ids == ["stage_1"]
    assert source.captures == 1


def test_continuous_workflow_release_quarantines_pending_calls(
    continuous_stack,
):
    _, controllers = continuous_stack
    workflow, _, execution = _continuous_workflow(continuous_stack)
    controllers[-1].pending = True
    controllers[-1].error = ObservationError("camera lost")

    workflow.run(_stage())

    owner, cleanup = execution.released[0]
    assert owner is workflow
    assert cleanup() is True


@pytest.mark.parametrize(
    ("error", "status"),
    [
        (ProtocolError("planner stopped"), FreeNavigationStatus.MODEL_ERROR),
        (
            ExecutionError("motion tracking failed"),
            FreeNavigationStatus.EXECUTION_ERROR,
        ),
        (FreeNavigationStalled("no progress"), FreeNavigationStatus.STUCK),
        (
            ObservationError("camera lost"),
            FreeNavigationStatus.OBSERVATION_ERROR,
        ),
    ],
)
def test_continuous_workflow_maps_failures_and_reports_measurements(
    continuous_stack, error, status
):
    windows, controllers = continuous_stack
    workflow, _, execution = _continuous_workflow(continuous_stack)
    controllers[-1].error = error

    result = workflow.run(_stage())

    assert result.success is False
    assert result.value.status is status
    assert str(error) in result.error
    assert result.value.planning_calls == 3
    assert result.value.traveled_m == 12.0
    assert result.value.final_pose == START.offset(12.0, 0.0)
    assert execution.stops == 1
    assert windows[-1].ends == 1


def test_continuous_workflow_requires_a_stopped_initial_frame(
    continuous_stack,
):
    windows, controllers = continuous_stack
    workflow, _, execution = _continuous_workflow(
        continuous_stack, source=FakeSource(_observation(stopped=False))
    )

    result = workflow.run(_stage())

    assert result.value.status is FreeNavigationStatus.OBSERVATION_ERROR
    assert "stopped observation" in result.error
    assert controllers[-1].calls == []
    assert windows[-1].ends == 1
    assert execution.stops == 1


def test_continuous_prepare_rejects_a_closed_workflow(continuous_stack):
    windows, controllers = continuous_stack
    workflow, _, _ = _continuous_workflow(continuous_stack)
    workflow.close()

    with pytest.raises(ExecutionError):
        workflow.prepare()

    assert controllers[-1].closed == 1
    assert windows[-1].closed == 1