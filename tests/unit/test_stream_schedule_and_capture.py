"""Five-second scheduling and image-time body pose, without simulator access."""
import io
import math
import sys
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest
from PIL import Image

from planner.adapters.airsim.observation_source import AirSimObservationSource
from planner.domain.motion import MotionLimits
from planner.domain.sliding_window import SlidingWindowSettings
from planner.services.planning_window import PlanningWindow
from planner.services.trajectory_validation import TrajectoryValidator
from tests.fakes.visual_goal import Clock, ForwardPlanner
from tests.unit.test_streaming_contracts import batch


def test_time_primary_schedule_does_not_wait_for_three_points_and_does_not_overlap_requests():
    clock=Clock()
    w=PlanningWindow(ForwardPlanner(),TrajectoryValidator(5,30,180),SlidingWindowSettings(enabled=True),MotionLimits(),clock)
    w.begin(deadline_monotonic=clock()+600,maximum_planning_rounds=30)
    try:
        w.queue.load_batch(batch())
        w.last_stream_submit=clock()
        clock.sleep(4.99)
        assert not w.stream_due()
        clock.sleep(.01)
        assert len(w.queue.items)==5 and w.stream_due()
        w.background.job=(None,Future())
        clock.sleep(7)
        assert not w.stream_due()
        w.background.job=None
        assert w.stream_due()  # no extra five-second delay after a slow result
        w.stream_retry_at=clock()+1
        assert not w.stream_due()
        clock.sleep(1)
        assert w.stream_due()
    finally:w.close()


def test_empty_queue_recovery_can_bypass_interval_but_not_retry_backoff():
    clock=Clock()
    w=PlanningWindow(ForwardPlanner(),TrajectoryValidator(5,30,180),SlidingWindowSettings(enabled=True),MotionLimits(),clock)
    w.begin(deadline_monotonic=clock()+600,maximum_planning_rounds=30)
    try:
        w.last_stream_submit=clock()
        assert w.stream_due()
        w.stream_retry_at=clock()+1
        assert not w.stream_due()
    finally:w.close()


@pytest.mark.parametrize('value',[0,-1,True,float('nan'),float('inf')])
def test_interval_requires_positive_finite_seconds(value):
    with pytest.raises(ValueError):SlidingWindowSettings(planning_interval_s=value)


def test_moving_capture_uses_image_pose_and_mount_without_reading_live_vehicle_or_hover(monkeypatch):
    monkeypatch.setitem(sys.modules,'airsim',NS(ImageRequest=lambda *a:a,ImageType=NS(Scene=0,DepthPerspective=2)))
    buffer=io.BytesIO();Image.new('RGB',(8,4)).save(buffer,format='PNG')
    q=NS(x_val=0.,y_val=0.,z_val=math.sqrt(.5),w_val=math.sqrt(.5))
    p=NS(x_val=3.,y_val=4.,z_val=-2.)
    common=dict(time_stamp=1000000000,camera_position=p,camera_orientation=q)
    rgb=NS(**common,image_data_uint8=buffer.getvalue(),width=8,height=4)
    depth=NS(**common,image_data_float=[5.]*64,width=8,height=8)
    class Connection:
        def call(self,name,*args):
            if name=='simGetImages':return [rgb,depth]
            if name=='simGetCameraInfo':return NS(fov=90.)
            raise AssertionError(name)
    source=AirSimObservationSource(Connection(),object(),'front_center',.5,200,
        verify_image_pair=True,depth_horizontal_fov_deg=90,camera_mount_position_body_m=(1,0,0))
    frame=source.capture_moving()
    assert not frame.stopped
    assert (frame.vehicle_pose.x,frame.vehicle_pose.y,frame.vehicle_pose.z)==pytest.approx((3,3,-2))
    assert frame.vehicle_pose.yaw_deg==pytest.approx(90)
    assert frame.timestamp_ns==1000000000
    assert frame.rgb.size==(8,4) and frame.depth_image.size==(8,8)


def test_delayed_planning_frame_uses_only_history_at_its_capture_time():
    from planner.domain.execution import ExecutionRecord
    from planner.domain.pose import WorldPose, RelativePoseDelta
    from planner.domain.trajectory import RelativeTrajectory
    from planner.domain.planning_window import PlanningWindowInput
    from planner.domain.streaming import QueueObservationAnchor
    from tests.fakes.visual_goal import wall_observation
    clock=Clock()
    w=PlanningWindow(ForwardPlanner(),TrajectoryValidator(5,30,180),SlidingWindowSettings(enabled=True),MotionLimits(),clock)
    w.begin(deadline_monotonic=clock()+600,maximum_planning_rounds=30)
    start=WorldPose(0,0,-4,0)
    before_frame=WorldPose(1,0,-4,0)
    frame_pose=WorldPose(1.1,0,-4,0)
    history=ExecutionRecord(start,before_frame,RelativeTrajectory((RelativePoseDelta(1,0,0,0),)))
    # These later actions must never appear alongside the delayed frame.
    w.record=ExecutionRecord(start,WorldPose(3,0,-4,0),RelativeTrajectory((RelativePoseDelta(3,0,0,0),)))
    frame=wall_observation(20,frame_pose,clock=clock)
    anchor=QueueObservationAnchor(w.background.stage_token,w.queue.revision,frame.timestamp_ns,(),history)
    try:
        assert w.submit_stream(frame,None,'forward',PlanningWindowInput(start,45,45),anchor,lambda:None,lambda:None)
        snapshot=w.background.job[0]
        assert sum(p.dx for p in snapshot.history.points)==pytest.approx(1.1)
        assert snapshot.history_start==start
    finally:w.close()
