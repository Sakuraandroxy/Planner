import pytest

from planner.domain.pose import WorldPose, wrap_yaw_deg
from planner.domain.trajectory import MotionSegment, MotionTrajectory
from planner.errors import ExecutionError
from planner.services.execution import ExecutionService
from tests.fakes.components import FakeVehicle


def test_execution_failure_cancels_and_hovers():
    vehicle = FakeVehicle(fail=True)
    trajectory = MotionTrajectory((MotionSegment(
        WorldPose(0, 0, 0, 0), WorldPose(1, 0, 0, 0), "synchronized_quintic"
    ),))
    with pytest.raises(ExecutionError):
        ExecutionService(vehicle).execute(trajectory)
    assert vehicle.cancelled and vehicle.hovered


@pytest.mark.parametrize("angle", [180, -180])
def test_failure_without_pause_condition_keeps_actual_signed_turn_and_remaining_goal(angle):
    vehicle = FakeVehicle()
    vehicle.pose = WorldPose(0, 0, -4, 170)
    start = vehicle.pose
    segment = MotionSegment(start, WorldPose(0, 0, -4, wrap_yaw_deg(170+angle)),
                            "synchronized_quintic", yaw_delta_deg=angle)

    def interrupted(segment, *, budget=None, pose_checkpoint=None):
        for delta in (angle/8, angle/4, angle/2):
            unwrapped = start.yaw_deg+delta
            vehicle.pose = WorldPose(0, 0, -4, wrap_yaw_deg(unwrapped))
            assert not pose_checkpoint(vehicle.pose, unwrapped)
        raise ExecutionError("controller interrupted")

    vehicle.execute_segment = interrupted
    with pytest.raises(ExecutionError) as captured:
        ExecutionService(vehicle).execute_with_receipt(MotionTrajectory((segment,)))
    receipt = captured.value.receipt
    assert receipt.measured_actions[0].dyaw_deg == pytest.approx(angle/2)
    assert receipt.completed_segments == 0
    assert receipt.remaining.segments[0].end == segment.end
    assert receipt.remaining.segments[0].yaw_delta_deg == pytest.approx(angle/2)
    assert vehicle.cancelled and vehicle.hovered


def test_failure_receipt_keeps_only_unfinished_world_segments_and_actual_partial_motion():
    vehicle = FakeVehicle()
    poses = tuple(WorldPose(x, 0, 0, 0) for x in range(4))
    trajectory = MotionTrajectory(tuple(MotionSegment(a, b, "synchronized_quintic")
                                        for a, b in zip(poses, poses[1:])))
    execute = vehicle.execute_segment
    def fail_mid_segment(segment, **kwargs):
        if segment.end.x == 2:
            vehicle.pose = WorldPose(1.5, 0, 0, 0)
            raise RuntimeError("controller unavailable")
        execute(segment, **kwargs)
    vehicle.execute_segment = fail_mid_segment
    with pytest.raises(ExecutionError) as captured:
        ExecutionService(vehicle).execute_with_receipt(trajectory, pause_when=lambda pose: False)
    receipt = captured.value.receipt
    assert receipt.completed_segments == 1 and len(receipt.remaining.segments) == 2
    assert receipt.remaining.segments[0].start.x == 1.5 and receipt.remaining.segments[0].end.x == 2
    assert sum(action.dx for action in receipt.measured_actions) == 1.5
    assert vehicle.cancelled and vehicle.hovered

