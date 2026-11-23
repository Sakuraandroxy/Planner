"""流执行的控制器：一拍一条命令，返回即交还任务层。"""

import pytest

from planner.domain import (
    HolonomicCommand,
    MotionLimits,
    StopResult,
    TimedVehicleState,
    WorldPose,
)
from planner.errors import BudgetExceeded, ExecutionError
from planner.runtime import ControlLoop

LIMITS = MotionLimits(control_hz=20.0)


class ScriptedSource:
    """按脚本给出期望命令，并记录拿到的实测偏航。"""

    def __init__(self, commands):
        self.commands = list(commands)
        self.yaws = []

    def next(self, state, unwrapped_yaw_deg):
        self.yaws.append(unwrapped_yaw_deg)
        return self.commands.pop(0) if self.commands else None


class StreamingPlant:
    def __init__(self, yaw_deg=0.0):
        self.pose = WorldPose(0.0, 0.0, 0.0, yaw_deg)
        self.timestamp_ns = 1_000_000
        self.commands = []

    def stream_state(self):
        return TimedVehicleState(self.pose, self.timestamp_ns, 0.0, (0.0, 0.0, 0.0))

    def stream_command(self, command, duration_s):
        self.commands.append((command, duration_s))

    def current_pose(self):
        return self.pose

    def cancel(self):
        pass

    def hold(self):
        pass


class PassiveExecution:
    def __init__(self, *, deadline_error=None):
        self.stops = 0
        self._deadline_error = deadline_error

    def assert_owner(self):
        pass

    def checkpoint(self):
        if self._deadline_error is not None:
            raise self._deadline_error

    def check_deadline(self):
        pass

    def stop(self):
        self.stops += 1
        return StopResult(False)


class FakeQueue:
    """时间线只读这两个字段。"""

    items = ()
    revision = 0


def loop_for(plant, execution=None):
    loop = ControlLoop(
        execution or PassiveExecution(), plant, limits=LIMITS, timeout_s=10.0
    )
    loop.recorder.begin(FakeQueue(), "stage")
    return loop


def test_stream_sends_one_command_per_call_and_stops_when_goals_run_out():
    plant = StreamingPlant()
    execution = PassiveExecution()
    loop = loop_for(plant, execution)
    goal = HolonomicCommand(1.0, 0.0, 0.0, 0.0)
    source = ScriptedSource([goal])

    loop.run_stream(source, loop.stream_sink())
    assert plant.commands == [(goal, pytest.approx(0.05))]
    assert execution.stops == 0

    # 目标源没有命令：正常返回，并停一次；再调一次不重复停。
    loop.run_stream(source, loop.stream_sink())
    loop.run_stream(source, loop.stream_sink())
    assert len(plant.commands) == 1
    assert execution.stops == 1


def test_stream_rejects_an_expired_deadline_before_sending():
    plant = StreamingPlant()
    loop = loop_for(plant)
    source = ScriptedSource([HolonomicCommand(1.0, 0.0, 0.0, 0.0)])
    with pytest.raises(BudgetExceeded):
        loop.run_stream(
            source,
            loop.stream_sink(),
            deadline_monotonic=-1.0,
        )
    assert plant.commands == []


def test_stream_unwraps_yaw_across_the_pi_boundary():
    plant = StreamingPlant(yaw_deg=179.0)
    loop = loop_for(plant)
    source = ScriptedSource([HolonomicCommand(0.0, 0.0, 0.0, 0.0)])
    loop.run_stream(source, loop.stream_sink())
    plant.pose = WorldPose(0.0, 0.0, 0.0, -179.0)
    plant.timestamp_ns += 1
    loop.run_stream(source, loop.stream_sink())
    assert source.yaws == [pytest.approx(179.0), pytest.approx(181.0)]


def test_stream_rejects_a_clock_that_moved_backwards():
    plant = StreamingPlant()
    loop = loop_for(plant)
    source = ScriptedSource([])
    plant.timestamp_ns = 10
    loop.run_stream(source, loop.stream_sink())
    plant.timestamp_ns = 5
    with pytest.raises(ExecutionError, match="backwards"):
        loop.run_stream(source, loop.stream_sink())

