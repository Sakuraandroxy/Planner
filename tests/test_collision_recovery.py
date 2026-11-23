"""碰撞退避：第 n 次后退的距离、预算上限、进度复位与恢复期间不连乘。

用积分位姿的假载具注入接触事件，断言执行层自己的后退距离；任务层不再有第二份
镜像实现，所以这里也是这些行为的唯一入口。
"""

import pytest

from planner.domain import (
    CollisionContact,
    CollisionRecoverySettings,
    CollisionState,
    HolonomicCommand,
    MotionCapability,
    MotionLimits,
    MotionModel,
    MotionSegment,
    MotionTrajectory,
    StopResult,
    TimedVehicleState,
    WorldPose,
)
from planner.errors import ExecutionError
from planner.runtime import SMOOTH_PROFILE, ControlLoop
from planner.runtime.collision_recovery import CollisionRecovery

LIMITS = MotionLimits(
    max_speed_mps=1.0,
    max_acceleration_mps2=5.0,
    control_hz=20.0,
    position_tolerance_m=0.1,
    yaw_tolerance_deg=1.0,
    stopped_speed_mps=0.15,
    stopped_yaw_rate_deg_s=5.0,
    settle_time_s=0.1,
    min_duration_s=0.2,
)

CAPABILITY = MotionCapability(
    MotionModel.HOLONOMIC,
    max_lateral_speed_mps=2.0,
    min_turn_radius_m=None,
    allows_reverse=True,
)

SETTINGS = CollisionRecoverySettings(
    backoff_m=1.0, maximum_recoveries=3, reset_after_progress_m=5.0
)

AT_REST = WorldPose(0.0, 0.0, 0.0, 0.0)


class Wall:
    """把每条命令按一个周期积分进位姿的载具；接触事件由测试注入。"""

    def __init__(self):
        self.pose = AT_REST
        self.pending = None  # 读一次就消费，与适配层的端口语义一致
        self.latched = None  # 一直上报，模拟锁定到下一次复位
        self.commands = []
        self.timestamp_ns = 0

    def stream_state(self):
        self.timestamp_ns += 1_000_000
        return TimedVehicleState(self.pose, self.timestamp_ns, 0.0, (0.0, 0.0, 0.0))

    def stream_command(self, command, duration_s):
        self.commands.append((command, duration_s))
        self.pose = WorldPose(
            self.pose.x + command.vx * duration_s,
            self.pose.y + command.vy * duration_s,
            self.pose.z + command.vz * duration_s,
            self.pose.yaw_deg + command.yaw_rate_deg_s * duration_s,
        )

    def current_pose(self):
        return self.pose

    def motion_capability(self):
        return CAPABILITY

    def collision_state(self):
        if self.latched is not None:
            return CollisionState(True, self.latched)
        contact, self.pending = self.pending, None
        return CollisionState(contact is not None, contact)

    def cancel(self):
        pass

    def hold(self):
        pass


class PassiveExecution:
    """只提供循环要用的检查点与停止。"""

    def __init__(self):
        self.stops = 0

    def assert_owner(self):
        pass

    def checkpoint(self):
        pass

    def check_deadline(self):
        pass

    def stop(self):
        self.stops += 1
        return StopResult(False)


class FakeWindow:
    def __init__(self):
        self.interrupts = 0

    def interrupt(self):
        self.interrupts += 1


class FakeQueue:
    """时间线只读这两个字段。"""

    items = ()
    revision = 0


class ForwardGoal:
    """一拍一条前进命令；恢复那一拍不会被调用。"""

    def next(self, state, unwrapped_yaw_deg):
        return HolonomicCommand(0.5, 0.0, 0.0, 0.0)


def loop_for(vehicle, *, settings=SETTINGS, window=None):
    loop = ControlLoop(
        PassiveExecution(),
        vehicle,
        collision=settings,
        window=window,
        limits=LIMITS,
        timeout_s=10.0,
    )
    loop.recorder.begin(FakeQueue(), "stage")
    return loop


def test_each_recovery_backs_off_one_backoff_further():
    vehicle = Wall()
    loop = loop_for(vehicle)
    goal = ForwardGoal()
    for attempt in (1, 2, 3):
        vehicle.pending = CollisionContact("Wall", float(attempt))
        before = vehicle.pose.x
        loop.run_stream(goal, loop.stream_sink())
        assert vehicle.pose.x - before == pytest.approx(-1.0 * attempt, abs=0.05)


def test_budget_is_exhausted_after_the_configured_recoveries():
    vehicle = Wall()
    loop = loop_for(
        vehicle,
        settings=CollisionRecoverySettings(
            backoff_m=1.0, maximum_recoveries=2, reset_after_progress_m=5.0
        ),
    )
    for attempt in (1, 2):
        vehicle.pending = CollisionContact("Wall", float(attempt))
        loop.run_stream(ForwardGoal(), loop.stream_sink())
    assert loop.recovery.attempts == 2
    vehicle.pending = CollisionContact("Wall", 3.0)
    with pytest.raises(ExecutionError, match="collision recovery budget exhausted"):
        loop.run_stream(ForwardGoal(), loop.stream_sink())


def test_progress_clears_the_attempt_count():
    recovery = CollisionRecovery(SETTINGS)
    contact = CollisionContact("Wall", 1.0)
    recovery.poll(CollisionState(True, contact))
    recovery.begin(contact, AT_REST)
    recovery.finish()
    assert recovery.attempts == 1
    # 基线在恢复后的第一拍取，复位判据是"恢复之后又前进了多少"。
    recovery.note_progress(10.0)
    recovery.note_progress(10.0 + SETTINGS.reset_after_progress_m - 0.1)
    assert recovery.attempts == 1
    recovery.note_progress(10.0 + SETTINGS.reset_after_progress_m)
    assert recovery.attempts == 0


def test_a_contact_during_recovery_is_not_counted_twice():
    recovery = CollisionRecovery(SETTINGS)
    first = CollisionContact("Wall", 1.0)
    recovery.begin(first, AT_REST)
    # 倒车时同一次物理接触会连续上报，新事件也不能算第二次恢复。
    assert recovery.poll(CollisionState(True, first)) is None
    assert recovery.poll(CollisionState(True, CollisionContact("Wall", 2.0))) is None
    recovery.finish()
    assert recovery.attempts == 1
    # 恢复结束后，触发它的那个事件也不再消费。
    assert recovery.poll(CollisionState(True, first)) is None
    assert recovery.poll(CollisionState(True, CollisionContact("Wall", 3.0))) is not None


def test_a_latched_contact_recovers_once():
    vehicle = Wall()
    vehicle.latched = CollisionContact("Wall", 1.0)
    loop = loop_for(vehicle)
    for _ in range(3):
        loop.run_stream(ForwardGoal(), loop.stream_sink())
    assert loop.recovery.attempts == 1


def test_a_contact_on_the_stream_clears_the_planning_queue():
    vehicle = Wall()
    window = FakeWindow()
    loop = loop_for(vehicle, window=window)
    vehicle.pending = CollisionContact("Wall", 1.0)
    loop.run_stream(ForwardGoal(), loop.stream_sink())
    assert window.interrupts == 1
    assert vehicle.pose.x == pytest.approx(-1.0, abs=0.05)


def test_a_contact_inside_a_batch_returns_the_remaining_trajectory():
    vehicle = Wall()
    window = FakeWindow()
    loop = loop_for(vehicle, window=window)
    vehicle.pending = CollisionContact("Wall", 1.0)
    segment = MotionSegment(
        start=AT_REST,
        end=WorldPose(2.0, 0.0, 0.0, 0.0),
        profile=SMOOTH_PROFILE,
        target_speed_mps=1.0,
        yaw_delta_deg=0.0,
    )
    receipt = loop.run_segment(MotionTrajectory((segment,)), loop.segment_sink())
    assert receipt.paused is False
    assert receipt.completed_segments == 0
    assert len(receipt.remaining.segments) == 1
    assert receipt.remaining.segments[0].start == receipt.final_pose
    assert receipt.final_pose.x < 0
    # 段执行的后退不清 authorized 队列。
    assert window.interrupts == 0
