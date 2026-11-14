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
from planner.capabilities.task_validation.free_navigation import (
    FreeNavigationStageValidator,
)
from planner.domain.basic_motion import BasicMotionParameters
from planner.domain.execution import ExecutionBudget, ExecutionRecord, StopResult
from planner.domain.free_navigation import (
    FreeNavigationParameters,
    FreeNavigationStatus,
)
from planner.domain.lane_reference import LaneLookaheadPoint, LaneReference
from planner.domain.mission import MissionStage, TaskKind
from planner.domain.motion import MotionLimits
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.sliding_window import QueuedWaypoint, SlidingWindowSettings
from planner.domain.streaming import QueueObservationAnchor
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.trajectory_telemetry import MODE_CONTINUOUS
from planner.errors import (
    ExecutionError,
    FreeNavigationStalled,
    ObservationError,
    ProtocolError,
)
from planner.workflows.free_navigation import FreeNavigationWorkflow
from planner.workflows.free_navigation.continuous import (
    FreeNavigationContinuous,
)

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


def _lane_reference():
    return LaneReference(
        road_id=45,
        lane_id=-4,
        lane_width_m=3.5,
        lateral_offset_m=2.47,
        heading_error_deg=-47.4,
        distance_to_left_boundary_m=4.22,
        distance_to_right_boundary_m=-0.72,
        lookahead=(
            LaneLookaheadPoint(
                distance_m=5.0,
                forward_m=4.2,
                right_m=-2.7,
                bearing_deg=-32.7,
                yaw_delta_deg=-40.0,
            ),
        ),
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
        source, planner, validator, motion, execution, **kwargs
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
    assert "Never apply the pending turn a second time" in prompt
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
    assert "Lane reference" in text
    assert "unavailable at the planning anchor" in text
    with pytest.raises(ProtocolError):
        prompt.build_user_prompt(
            "follow the road", _observation(), 5, task_context=object()
        )


def test_user_prompt_formats_a_measured_lane_reference():
    prompt = FreeNavigationTrajectoryPrompt()
    text = prompt.build_user_prompt(
        "follow the road",
        _observation(),
        5,
        task_context=_lane_reference(),
    )
    assert "Lane reference (measured from the simulator HD map" in text
    assert "road 45, lane -4" in text
    assert "width 3.50 m" in text
    assert "Lateral offset: +2.47 m" in text
    assert "Lane heading error: -47.4 deg" in text
    assert "left lane boundary: 4.22 m" in text
    assert "right lane boundary: -0.72 m" in text
    assert "cumulative lane yaw -40.0 deg" in text
    assert "Steering aim at the anchor" in text
    assert "lane-centre point is at bearing -32.7 deg" in text
    assert "Final-heading check" in text
    assert "within about 5 deg of the cumulative lane yaw at the reach" in text
    assert "near bearing is only the early aim" in text
    assert "first one or two actions must turn toward the" in text
    assert "Tight space at the anchor" in text
    assert "short pivot-like steps" in text
    assert "Open lane at the anchor" not in text
    assert "already includes the effect of the pending actions" in text
    assert "Metric free-space profile" in text


def test_user_prompt_uses_normal_steps_on_an_open_lane():
    reference = LaneReference(
        road_id=45,
        lane_id=-4,
        lane_width_m=3.5,
        lateral_offset_m=0.1,
        heading_error_deg=1.0,
        distance_to_left_boundary_m=1.7,
        distance_to_right_boundary_m=1.8,
        lookahead=(
            LaneLookaheadPoint(
                distance_m=6.0,
                forward_m=6.0,
                right_m=0.17,
                bearing_deg=1.6,
                yaw_delta_deg=1.0,
            ),
        ),
    )
    text = FreeNavigationTrajectoryPrompt().build_user_prompt(
        "follow the road",
        _observation(),
        5,
        task_context=reference,
    )
    assert "Open lane at the anchor" in text
    assert "Use normal 4-5 m forward steps" in text
    assert "Tight space at the anchor" not in text


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


def test_workflow_supplies_the_measured_lane_reference_to_every_plan():
    reference = _lane_reference()
    poses = []

    class Provider:
        def lane_reference(self, pose):
            poses.append(pose)
            return reference

    planner = FakePlanner(
        _trajectory(), fail_at=2, error=ProtocolError("planner stopped")
    )
    workflow, _, _, _, _, _ = _workflow(
        planner=planner, lane_reference_provider=Provider()
    )

    workflow.run(_stage())

    assert poses == [START, START]
    assert all(
        call[2]["task_context"] is reference for call in planner.calls
    )


def test_workflow_lane_reference_failure_degrades_to_unavailable():
    class Provider:
        def lane_reference(self, pose):
            raise RuntimeError("map projection failed")

    planner = FakePlanner(
        _trajectory(), fail_at=1, error=ProtocolError("planner stopped")
    )
    workflow, _, _, _, _, _ = _workflow(
        planner=planner, lane_reference_provider=Provider()
    )

    workflow.run(_stage())

    assert planner.calls[0][2]["task_context"] is None


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


class FakeContinuousStream:
    def __init__(self, *, stop_after=None):
        self.limits = SimpleNamespace(control_hz=20.0)
        self.state = SimpleNamespace(pose=START)
        self.record = ExecutionRecord(START, START, RelativeTrajectory(()))
        self.path_length_m = 0.0
        self.advance_per_tick = 0.0
        self.ticks = 0
        self.stop_after = stop_after
        self.begun = []
        self.timeline = FakeTimeline()
        self.follower = SimpleNamespace(reset=lambda *args: None)
        self.goal_yaw = None
        self.head_id = None
        self.paused = None
        self.holding = True

    def begin(self, queue, stage_token):
        self.begun.append((queue, stage_token))

    def sample(self):
        pass

    def tick(self, pause_reason, *, sample=True):
        self.ticks += 1
        if self.stop_after is not None and self.ticks > self.stop_after:
            raise KeyboardInterrupt
        self.path_length_m += self.advance_per_tick


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
        return None

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
            stream_velocity=lambda *args: None,
            collision_state=lambda: False,
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
    lane_reference_provider=None,
):
    stream = stream or FakeContinuousStream()
    frames = frames or FakeMovingFrames()
    clock = clock or FakeClock()
    monkeypatch.setattr(
        free_continuous_module,
        "ContinuousExecution",
        lambda *args, **kwargs: stream,
    )
    monkeypatch.setattr(
        free_continuous_module,
        "MovingObservations",
        lambda *args, **kwargs: frames,
    )
    execution = FakeStreamingExecution(vehicle=vehicle)
    controller = FreeNavigationContinuous(
        execution,
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
        lane_reference_provider=lane_reference_provider,
        telemetry=telemetry,
    )
    return controller, execution, stream, frames


def test_continuous_loop_submits_stopped_and_moving_frames(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousStream(stop_after=2)
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
    assert stream.begun == [(window.queue, window.background.stage_token)]
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


def test_continuous_loop_attaches_the_measured_lane_reference(monkeypatch):
    reference = _lane_reference()
    poses = []

    class Provider:
        def lane_reference(self, pose):
            poses.append(pose)
            return reference

    moving = _observation(
        stopped=False, capture_id="moving", timestamp_ns=2
    )
    stream = FakeContinuousStream(stop_after=2)
    frames = FakeMovingFrames(frames=[moving], delay=1)
    controller, _, stream, frames = _continuous_controller(
        monkeypatch,
        stream=stream,
        frames=frames,
        clock=FakeClock(),
        lane_reference_provider=Provider(),
    )
    window = FakePlanningWindow()

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
        )

    assert poses == [START, START]
    assert all(item[1] is reference for item in window.submissions)


def test_continuous_lane_reference_uses_the_pending_queue_tail(monkeypatch):
    reference = _lane_reference()
    tail = WorldPose(2.0, 0.0, 0.0, 10.0)
    pending = QueuedWaypoint(1, START, tail, 10.0)
    poses = []

    class Provider:
        def lane_reference(self, pose):
            poses.append(pose)
            return reference

    class PendingTimeline:
        def at(self, timestamp_ns, stage_token):
            return QueueObservationAnchor(
                stage_token, 3, timestamp_ns, (pending,)
            )

    stream = FakeContinuousStream(stop_after=1)
    stream.timeline = PendingTimeline()
    controller, _, _, _ = _continuous_controller(
        monkeypatch,
        stream=stream,
        clock=FakeClock(),
        lane_reference_provider=Provider(),
    )
    window = FakePlanningWindow()

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
        )

    assert poses == [tail]
    assert window.submissions[0][4].items == (pending,)


def test_continuous_lane_reference_failure_degrades_to_unavailable(monkeypatch):
    class Provider:
        def lane_reference(self, pose):
            raise RuntimeError("map projection failed")

    stream = FakeContinuousStream(stop_after=1)
    controller, _, _, _ = _continuous_controller(
        monkeypatch,
        stream=stream,
        clock=FakeClock(),
        lane_reference_provider=Provider(),
    )
    window = FakePlanningWindow()

    with pytest.raises(KeyboardInterrupt):
        controller.run(
            FreeNavigationParameters("follow the road"),
            _observation(),
            window,
        )

    assert window.submissions[0][1] is None


def test_continuous_collision_backs_off_clears_queue_and_replans(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousStream(stop_after=2)
    recovered = _observation(
        stopped=True, capture_id="recovered", timestamp_ns=3
    )
    source = FakeSource(recovered)
    # The first read drains a contact latched before the stage started; only
    # the second read is a contact made while driving.
    collisions = iter([False, True, False, False, False, False])
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_velocity=lambda *args: None,
        collision_state=lambda: next(collisions, False),
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
    assert stream.initial == START
    assert stream.actions == []
    assert window.submissions
    assert window.submissions[0][0].capture_id == "recovered"
    assert frames.discarded >= 1


def test_continuous_discards_a_contact_latched_before_the_stage(monkeypatch):
    clock = FakeClock()
    stream = FakeContinuousStream(stop_after=2)
    collisions = iter([True, False, False])
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_velocity=lambda *args: None,
        collision_state=lambda: next(collisions, False),
        hover=lambda: None,
        last_collision=SimpleNamespace(object_name="Rover1"),
    )
    controller, execution, stream, frames = _continuous_controller(
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
    stream = FakeContinuousStream(stop_after=10)
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_velocity=lambda *args: None,
        collision_state=lambda: True,
        hover=lambda: None,
    )
    controller, execution, stream, frames = _continuous_controller(
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
    stream = FakeContinuousStream()
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_velocity=lambda *args: None,
        collision_state=lambda: True,
        hover=lambda: None,
    )
    controller, execution, stream, frames = _continuous_controller(
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
    stream = FakeContinuousStream(stop_after=3)
    stream.advance_per_tick = 2.0
    # Each recovery reads collision_state once more to clear the latch; the
    # leading False is the pre-task drain read.
    collisions = iter([False, True, False, False, True, False])
    vehicle = SimpleNamespace(
        stream_state=lambda: None,
        stream_velocity=lambda *args: None,
        collision_state=lambda: next(collisions, False),
        hover=lambda: None,
    )
    controller, execution, stream, frames = _continuous_controller(
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
    stream = FakeContinuousStream(stop_after=1)
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
    stream = FakeContinuousStream()
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
    stream = FakeContinuousStream(stop_after=6)
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
    workflow, source, planner, validator, motion, execution = _workflow(
        **kwargs
    )
    return workflow, source, execution


def test_continuous_mode_requires_an_enabled_window_and_motion_limits():
    def build(**kwargs):
        return FreeNavigationWorkflow(
            FakeSource(_observation()),
            FakePlanner(_trajectory()),
            FakeValidator(),
            FakeMotion(),
            FakeExecution(),
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