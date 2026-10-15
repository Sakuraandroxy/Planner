"""Deterministic kinematic fake: tests tracking logic, not AirSim flight dynamics."""
import math
import sys
from types import SimpleNamespace

import pytest

from planner.adapters.airsim.motion_tracker import AirSimMotionTracker
from planner.adapters.airsim.observation_gate import ObservationGate
from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import WorldTrajectory
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.pose import RelativePoseDelta
from planner.services.trajectory_transform import relative_to_world
from planner.errors import ExecutionError, BudgetExceeded
from planner.domain.execution import ExecutionBudget
from planner.services.motion.synchronized import SynchronizedMotionPlanner


class SimulatedConnection:
    def __init__(self, start, moving=True, collision_after=None):
        self.pose = start
        self.time = 0.0
        self.moving = moving
        self.collision_after = collision_after
        self.commands = []
        self.hovered = False

    def sleep(self, seconds):
        self.time += seconds

    def call(self, name):
        assert name == "simGetCollisionInfo"
        return SimpleNamespace(has_collided=self.collision_after is not None and len(self.commands) >= self.collision_after)

    def call_async_and_wait(self, name, *args, **kwargs):
        if name == "hoverAsync":
            self.hovered = True
            return
        assert name == "moveByVelocityAsync"
        assert kwargs["drivetrain"] == "independent"
        assert kwargs["yaw_mode"].is_rate
        vx, vy, vz, dt = args
        rate = kwargs["yaw_mode"].yaw_or_rate
        self.commands.append((self.time, (vx, vy, vz), rate, self.pose))
        if self.moving:
            a = self.pose
            self.pose = WorldPose(a.x+vx*dt, a.y+vy*dt, a.z+vz*dt, wrap_yaw_deg(a.yaw_deg+rate*dt))
        self.time += dt


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    monkeypatch.setitem(sys.modules, "airsim", SimpleNamespace(
        DrivetrainType=SimpleNamespace(MaxDegreeOfFreedom="independent"),
        YawMode=lambda **kwargs: SimpleNamespace(**kwargs),
    ))


def run_motion(start, end, moving=True, collision_after=None, timeout=40):
    limits = MotionLimits()
    connection = SimulatedConnection(start, moving, collision_after)
    segment = SynchronizedMotionPlanner(2, limits).create_motion(WorldTrajectory(start, (end,))).segments[0]
    tracker = AirSimMotionTracker(connection, lambda: connection.pose, limits, timeout,
                                 clock=lambda: connection.time, sleep=connection.sleep)
    return connection, segment, tracker


@pytest.mark.parametrize("start_yaw", [0, 170, -170])
@pytest.mark.parametrize("angle", [180, -180])
def test_execution_receipt_preserves_measured_half_turn_without_pause_condition(start_yaw, angle):
    from planner.services.execution import ExecutionService
    start = WorldPose(0, 0, -5, start_yaw)
    connection = SimulatedConnection(start)
    world = relative_to_world(RelativeTrajectory((RelativePoseDelta(0, 0, 0, angle),)), start)
    motion = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(world)
    tracker = AirSimMotionTracker(connection, lambda: connection.pose, MotionLimits(), 40,
        clock=lambda: connection.time, sleep=connection.sleep)
    vehicle = SimpleNamespace(
        current_pose=lambda: connection.pose,
        execute_segment=lambda segment, **kwargs: tracker.execute(segment, **kwargs),
        collision_state=lambda: False, cancel=lambda: None,
        hover=lambda: connection.call_async_and_wait("hoverAsync"))
    receipt = ExecutionService(vehicle, clock=lambda: connection.time).execute_with_receipt(motion)
    measured_turn = sum(command[2]*0.05 for command in connection.commands)
    assert receipt.measured_actions[0].dyaw_deg == pytest.approx(measured_turn, abs=0.001)
    assert receipt.measured_actions[0].dyaw_deg == pytest.approx(angle, abs=2)
    assert not receipt.paused and receipt.completed_segments == 1


def test_execution_receipt_uses_measured_start_branch_when_start_crosses_yaw_boundary():
    from planner.services.execution import ExecutionService
    planned_start = WorldPose(0, 0, -5, 179)
    connection = SimulatedConnection(WorldPose(0, 0, -5, -179.9))
    world = relative_to_world(RelativeTrajectory((RelativePoseDelta(0, 0, 0, 20),)), planned_start)
    motion = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(world)
    tracker = AirSimMotionTracker(connection, lambda: connection.pose, MotionLimits(), 40,
        clock=lambda: connection.time, sleep=connection.sleep)
    vehicle = SimpleNamespace(current_pose=lambda: connection.pose,
        execute_segment=lambda segment, **kwargs: tracker.execute(segment, **kwargs),
        collision_state=lambda: False, cancel=lambda: None,
        hover=lambda: connection.call_async_and_wait("hoverAsync"))
    receipt = ExecutionService(vehicle, clock=lambda: connection.time).execute_with_receipt(motion)
    assert receipt.measured_actions[0].dyaw_deg == pytest.approx(18.9, abs=2)
    assert receipt.measured_actions[0].dyaw_deg == pytest.approx(
        sum(command[2]*0.05 for command in connection.commands), abs=0.001)


def test_task_pause_mid_segment_preserves_world_endpoints_and_signed_yaw_on_resume():
    from planner.services.execution import ExecutionService
    from planner.capabilities.completion.visual_goal_retained_motion import retained_world_trajectory
    from planner.domain.trajectory import MotionTrajectory
    start, end = WorldPose(0, 0, -5, 0), WorldPose(4, 0, -5, -180)
    connection = SimulatedConnection(start)
    motion = SynchronizedMotionPlanner(2, MotionLimits())
    original = motion.create_motion(WorldTrajectory(start, (end,), (180,)))
    tracker = AirSimMotionTracker(connection, lambda: connection.pose, MotionLimits(), 40,
        clock=lambda: connection.time, sleep=connection.sleep)
    vehicle = SimpleNamespace(
        current_pose=lambda: connection.pose,
        execute_segment=lambda segment, **kwargs: tracker.execute(segment, **kwargs),
        collision_state=lambda: False,
        cancel=lambda: None,
        hover=lambda: connection.call_async_and_wait("hoverAsync"),
    )
    execution = ExecutionService(vehicle, clock=lambda: connection.time)
    paused = execution.execute_with_receipt(original, pause_when=lambda pose: pose.x >= 1)
    assert paused.paused and paused.completed_segments == 0
    assert 1 <= paused.final_pose.x < 4 and connection.hovered
    assert paused.remaining.segments[0].end == end
    actual_yaw = paused.measured_actions[0].dyaw_deg
    assert 0 < actual_yaw < 180
    assert paused.remaining.segments[0].yaw_delta_deg == pytest.approx(180-actual_yaw)
    commands_at_pause = len(connection.commands)
    # No implicit replay happens while the decision is pending.
    assert len(connection.commands) == commands_at_pause
    resumed = motion.create_motion(retained_world_trajectory(paused.remaining, connection.pose))
    receipt = execution.execute_with_receipt(resumed)
    assert not receipt.paused and receipt.completed_segments == 1
    assert abs(wrap_yaw_deg(connection.pose.yaw_deg-end.yaw_deg)) <= 2
    assert abs(connection.pose.x-end.x) <= 0.3
    measured_total = sum(command[2]*0.05 for command in connection.commands)
    assert actual_yaw + receipt.measured_actions[0].dyaw_deg == pytest.approx(measured_total, abs=0.001)
    assert measured_total == pytest.approx(180, abs=2)


@pytest.mark.parametrize("start,end", [
    (WorldPose(0, 0, -5, 90), WorldPose(8, 5, -8, 180)),
    (WorldPose(1, 2, -5, 170), WorldPose(1, 2, -5, -170)),
    (WorldPose(0, 0, -5, 0), WorldPose(0, 0, -17, 0)),
])
def test_reaches_position_and_yaw_with_bounded_commands(start, end):
    connection, segment, tracker = run_motion(start, end)
    tracker.execute(segment)
    final = connection.pose
    assert math.dist((final.x, final.y, final.z), (end.x, end.y, end.z)) <= 0.3
    assert abs(wrap_yaw_deg(final.yaw_deg-end.yaw_deg)) <= 2
    assert connection.hovered
    for a, b in zip(connection.commands, connection.commands[1:]):
        dt = b[0]-a[0]
        assert math.dist(a[1], b[1])/dt <= 1.5 + 1e-6
        assert abs(a[2]-b[2])/dt <= 30 + 1e-6
        assert math.sqrt(sum(v*v for v in b[1])) <= 2 + 1e-6
        assert abs(b[2]) <= 30 + 1e-6
    if end.x != start.x:
        mid = min(connection.commands, key=lambda c: abs(c[0]-segment.duration_s/2))[3]
        position_fraction = (mid.x-start.x)/(end.x-start.x)
        yaw_fraction = wrap_yaw_deg(mid.yaw_deg-start.yaw_deg)/wrap_yaw_deg(end.yaw_deg-start.yaw_deg)
        assert abs(position_fraction-yaw_fraction) < 0.06
        assert 0.4 < position_fraction < 0.6


def test_stalled_vehicle_does_not_report_success():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(3, 0, -5, 45), moving=False, timeout=10)
    with pytest.raises(ExecutionError, match="timed out"):
        tracker.execute(segment)


def test_motion_rejects_budget_too_short_before_first_velocity_command():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(3, 0, -5, 0))
    with pytest.raises(BudgetExceeded):
        tracker.execute(segment, budget=ExecutionBudget(1))
    assert not connection.commands


def test_collision_checked_during_segment():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(3, 0, -5, 45), collision_after=5)
    from planner.services.execution import ExecutionService
    execution = ExecutionService(SimpleNamespace(), clock=lambda: connection.time)
    execution.acquire_flight(tracker, collision_check=lambda: bool(connection.call("simGetCollisionInfo").has_collided))
    def checkpoint(pose, yaw):
        execution.check_safety()
        return False
    with pytest.raises(ExecutionError, match="collision"):
        tracker.execute(segment, pose_checkpoint=checkpoint)
    assert len(connection.commands) == 5
    execution.release_flight(tracker)


def test_duration_over_timeout_rejected_before_motion():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(30, 0, -5, 90), timeout=5)
    with pytest.raises(ExecutionError, match="duration"):
        tracker.execute(segment)
    assert not connection.commands


def test_observation_mid_segment_resumes_original_goal():
    start, end = WorldPose(0, 0, -5, 0), WorldPose(8, 2, -5, 90)
    connection, segment, tracker = run_motion(start, end)
    vehicle = SimpleNamespace(
        hover=lambda: connection.call_async_and_wait("hoverAsync"),
        current_pose=lambda: connection.pose,
        collision_state=lambda: False,
    )
    gate = ObservationGate(vehicle, MotionLimits(), 40,
                           clock=lambda: connection.time, sleep=connection.sleep)
    tracker.observation_gate = gate
    captured = []

    def sleep(seconds):
        connection.sleep(seconds)
        if len(connection.commands) >= 10 and not captured:
            with gate.capture():
                captured.append(connection.pose)
                connection.sleep(50)  # Observation wait must not consume flight timeout.

    tracker.sleep = sleep
    tracker.execute(segment)
    assert len(captured) == 1
    assert 0 < captured[0].x < end.x
    assert math.dist((connection.pose.x, connection.pose.y, connection.pose.z),
                     (end.x, end.y, end.z)) <= 0.3
    assert abs(wrap_yaw_deg(connection.pose.yaw_deg - end.yaw_deg)) <= 2


def test_failed_capture_blocks_further_motion():
    connection, segment, tracker = run_motion(WorldPose(0, 0, -5, 0), WorldPose(3, 0, -5, 0))
    vehicle = SimpleNamespace(hover=lambda: None, current_pose=lambda: connection.pose,
                              collision_state=lambda: False)
    gate = ObservationGate(vehicle, MotionLimits(), 40,
                           clock=lambda: connection.time, sleep=connection.sleep)
    with pytest.raises(RuntimeError):
        with gate.capture():
            raise RuntimeError("image unavailable")
    tracker.observation_gate = gate
    with pytest.raises(ExecutionError, match="observation failed"):
        tracker.execute(segment)
    assert not connection.commands


@pytest.mark.parametrize("angle", [180, -180])
@pytest.mark.parametrize("pause", [False, True])
def test_signed_half_turn_direction_and_pause_do_not_replay_rotation(angle, pause):
    start = WorldPose(0, 0, -5, 170)
    limits = MotionLimits()
    connection = SimulatedConnection(start)
    world = relative_to_world(RelativeTrajectory((RelativePoseDelta(0, 0, 0, angle),)), start)
    segment = SynchronizedMotionPlanner(2, limits).create_motion(world).segments[0]
    tracker = AirSimMotionTracker(connection, lambda: connection.pose, limits, 40,
                                 clock=lambda: connection.time, sleep=connection.sleep)
    captured = []
    if pause:
        vehicle = SimpleNamespace(hover=lambda: connection.call_async_and_wait("hoverAsync"),
                                  current_pose=lambda: connection.pose, collision_state=lambda: False)
        gate = ObservationGate(vehicle, limits, 40, clock=lambda: connection.time, sleep=connection.sleep)
        tracker.observation_gate = gate
        def sleep(seconds):
            connection.sleep(seconds)
            if len(connection.commands) >= 20 and not captured:
                with gate.capture():
                    captured.append(connection.pose)
                    connection.sleep(5)
        tracker.sleep = sleep
    tracker.execute(segment)
    # Integrate commands rather than just comparing normalized endpoints: a
    # right half-turn and a left half-turn end at the same wrapped yaw.
    total = sum(command[2]*0.05 for command in connection.commands)
    assert total == pytest.approx(angle, abs=2)
    first_rate = next(command[2] for command in connection.commands if abs(command[2]) > 0.01)
    assert first_rate*angle > 0
    assert all(math.hypot(*command[1]) == 0 for command in connection.commands)
    assert abs(wrap_yaw_deg(connection.pose.yaw_deg-world.poses[-1].yaw_deg)) <= 2
    assert len(captured) == int(pause)


def test_nonfinite_yaw_is_rejected_before_clamping_or_command(monkeypatch):
    import planner.adapters.airsim.motion_tracker as adapter
    connection = SimulatedConnection(WorldPose(0,0,-4,0))
    tracker = AirSimMotionTracker(connection, lambda: connection.pose, MotionLimits(), 60,
                                 lambda: connection.time, connection.sleep)
    motion = SynchronizedMotionPlanner(2, MotionLimits()).create_motion(
        WorldTrajectory(connection.pose, (WorldPose(1,0,-4,0),)))
    monkeypatch.setattr(adapter, "sample_segment", lambda *args: SimpleNamespace(
        pose=connection.pose, velocity=(0,0,0), yaw_rate_deg_s=float("nan")))
    with pytest.raises(ExecutionError, match="non-finite yaw"):
        tracker.execute(motion.segments[0])
    assert not connection.commands
