"""段执行的控制器：采样、限幅、停稳与预算都在 ControlLoop 里。"""

import math

import pytest

from planner.domain import (
    CollisionState,
    ExecutionBudget,
    MotionCapability,
    MotionLimits,
    MotionModel,
    MotionSegment,
    MotionTrajectory,
    StopResult,
    WorldPose,
)
from planner.errors import BudgetExceeded, ExecutionError
from planner.runtime import ControlLoop, SMOOTH_PROFILE
from planner.runtime.motion.tracking import limit_vector

LIMITS = MotionLimits(
    max_acceleration_mps2=5.0,
    max_yaw_rate_deg_s=90.0,
    max_yaw_acceleration_deg_s2=180.0,
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


class PerfectPlant:
    """把每条命令按一个周期积分进位姿的载具。"""

    def __init__(self, pose):
        self.pose = pose
        self.commands = []

    def current_pose(self):
        return self.pose

    def motion_capability(self):
        return CAPABILITY

    def collision_state(self):
        return CollisionState(False)

    def stream_command(self, command, duration_s):
        self.commands.append((command, duration_s))
        self.pose = WorldPose(
            self.pose.x + command.vx * duration_s,
            self.pose.y + command.vy * duration_s,
            self.pose.z + command.vz * duration_s,
            self.pose.yaw_deg + command.yaw_rate_deg_s * duration_s,
        )

    def cancel(self):
        pass

    def hold(self):
        pass


class FrozenPlant(PerfectPlant):
    def stream_command(self, command, duration_s):
        self.commands.append((command, duration_s))


class PassiveExecution:
    """只提供执行层检查点的执行服务替身。"""

    def __init__(self, *, stopped=False):
        self.stops = 0
        self._stopped = stopped

    def assert_owner(self):
        pass

    def checkpoint(self):
        if self._stopped:
            raise ExecutionError("flight stop requested")

    def check_deadline(self):
        pass

    def collision_state(self):
        return False

    def stop(self):
        self.stops += 1
        return StopResult(False)


def segment(start, end, *, speed=2.0, yaw_delta_deg=None):
    return MotionSegment(
        start=start,
        end=end,
        profile=SMOOTH_PROFILE,
        target_speed_mps=speed,
        yaw_delta_deg=yaw_delta_deg,
    )


def loop_for(plant, *, timeout_s=10.0, execution=None):
    loop = ControlLoop(
        execution or PassiveExecution(),
        plant,
        limits=LIMITS,
        timeout_s=timeout_s,
    )
    return loop


def test_limit_vector_scales_by_magnitude():
    assert limit_vector((3.0, 4.0), 10.0) == pytest.approx((3.0, 4.0))
    assert limit_vector((3.0, 0.0), 1.5) == pytest.approx((1.5, 0.0))
    assert limit_vector((0.0, 0.0), 3.0) == pytest.approx((0.0, 0.0))
    with pytest.raises(ExecutionError):
        limit_vector((3.0, 4.0), 0.0)
    with pytest.raises(ExecutionError):
        limit_vector((float("nan"), 4.0), 1.0)


def test_loop_drives_a_straight_segment_to_the_goal():
    plant = PerfectPlant(WorldPose(0.0, 0.0, 0.0, 0.0))
    loop = loop_for(plant)
    loop.run_segment(
        MotionTrajectory(
            (segment(WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(1.0, 0.0, 0.0, 0.0)),)
        ),
        loop.segment_sink(),
    )
    assert plant.pose.x == pytest.approx(1.0, abs=0.03)
    assert plant.pose.y == pytest.approx(0.0, abs=1e-9)
    assert plant.pose.yaw_deg == pytest.approx(0.0, abs=1e-6)
    # First command is at rest on the reference start.
    assert plant.commands[0][0].vx == pytest.approx(0.0)
    # Settling runs at least the configured settle window after the profile.
    assert len(plant.commands) >= math.ceil(0.9375 / LIMITS.segment_period_s)


def test_loop_reanchors_yaw_to_the_measured_heading():
    plant = PerfectPlant(WorldPose(0.0, 0.0, 0.0, 10.0))
    loop = loop_for(plant)
    loop.run_segment(
        MotionTrajectory(
            (
                segment(
                    WorldPose(0.0, 0.0, 0.0, 0.0), WorldPose(0.0, 0.0, 0.0, 90.0)
                ),
            )
        ),
        loop.segment_sink(),
    )
    assert plant.pose.yaw_deg == pytest.approx(90.0, abs=1.0)


def test_loop_requires_a_speed_limit():
    plant = PerfectPlant(WorldPose(0.0, 0.0, 0.0, 0.0))
    loop = loop_for(plant)
    with pytest.raises(ExecutionError, match="speed limit"):
        loop.run_segment(
            MotionTrajectory(
                (
                    MotionSegment(
                        start=plant.pose,
                        end=WorldPose(1.0, 0.0, 0.0, 0.0),
                        profile=SMOOTH_PROFILE,
                    ),
                )
            ),
            loop.segment_sink(),
        )
    assert plant.commands == []


def test_loop_pauses_at_a_pose_checkpoint():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(start)
    loop = loop_for(plant)
    seen = []

    def pause_when(pose):
        seen.append(pose)
        return pose.x > 0.4

    receipt = loop.run_segment(
        MotionTrajectory((segment(start, WorldPose(1.0, 0.0, 0.0, 0.0)),)),
        loop.segment_sink(),
        pause_when=pause_when,
    )
    assert receipt.paused is True
    assert receipt.final_pose.x > 0.4
    assert receipt.remaining.segments[0].start == receipt.final_pose
    assert receipt.remaining.segments[0].yaw_delta_deg == pytest.approx(
        0.0, abs=1e-9
    )
    assert seen


def test_loop_stops_when_a_stop_was_requested():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(start)
    loop = loop_for(plant, execution=PassiveExecution(stopped=True))
    with pytest.raises(ExecutionError, match="stop requested"):
        loop.run_segment(
            MotionTrajectory((segment(start, WorldPose(1.0, 0.0, 0.0, 0.0)),)),
            loop.segment_sink(),
        )
    assert plant.commands == []


def test_loop_rejects_an_exhausted_budget_before_moving():
    start = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(start)
    loop = loop_for(plant)
    budget = ExecutionBudget(deadline_monotonic=-1.0)
    with pytest.raises(BudgetExceeded):
        loop.run_segment(
            MotionTrajectory((segment(start, WorldPose(1.0, 0.0, 0.0, 0.0)),)),
            loop.segment_sink(),
            budget=budget,
        )
    assert plant.commands == []


def test_loop_times_out_when_the_rover_never_moves():
    pose = WorldPose(0.0, 0.0, 0.0, 0.0)
    frozen = FrozenPlant(pose)
    loop = loop_for(frozen, timeout_s=1.5)
    with pytest.raises(ExecutionError, match="timed out"):
        loop.run_segment(
            MotionTrajectory((segment(pose, WorldPose(1.0, 0.0, 0.0, 0.0)),)),
            loop.segment_sink(),
        )
    assert len(frozen.commands) >= math.ceil(1.5 / LIMITS.segment_period_s)


def test_loop_rejects_a_duration_beyond_the_available_timeout():
    pose = WorldPose(0.0, 0.0, 0.0, 0.0)
    plant = PerfectPlant(pose)
    loop = loop_for(plant, timeout_s=0.5)
    with pytest.raises(ExecutionError, match="execution timeout"):
        loop.run_segment(
            MotionTrajectory((segment(pose, WorldPose(10.0, 0.0, 0.0, 0.0), speed=1.0),)),
            loop.segment_sink(),
        )
    assert plant.commands == []
