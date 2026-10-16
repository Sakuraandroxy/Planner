"""Attitude provenance, buffered diagnostics and optional trace compatibility."""
import csv
import math
from types import SimpleNamespace as NS

import pytest

from planner.adapters.airsim.vehicle import AirSimVehicle
from planner.adapters.diagnostics.visual_goal_trace import VisualGoalTrace
from tests.integration.test_continuous_visual_goal import continuous_setup


def test_airsim_stream_state_supplies_pitch_roll_without_extra_rpc():
    roll, pitch, yaw = (math.radians(v) for v in (10,30,20))
    cr,sr,cp,sp,cy,sy = math.cos(roll/2),math.sin(roll/2),math.cos(pitch/2),math.sin(pitch/2),math.cos(yaw/2),math.sin(yaw/2)
    q=NS(w_val=cr*cp*cy+sr*sp*sy,x_val=sr*cp*cy-cr*sp*sy,
         y_val=cr*sp*cy+sr*cp*sy,z_val=cr*cp*sy-sr*sp*cy)
    vector=NS(x_val=1.,y_val=2.,z_val=-3.)
    class Connection:
        def __init__(self):self.calls=[]
        def call(self,name):
            self.calls.append(name)
            return NS(timestamp=1_000_000_000,kinematics_estimated=NS(
                orientation=q,position=vector,linear_velocity=vector))
    connection=Connection()
    state=AirSimVehicle(connection,2,60).stream_state()
    assert (state.roll_deg,state.pitch_deg,state.pose.yaw_deg)==pytest.approx((10,30,20))
    assert state.velocity_world==(1,2,-3)
    assert connection.calls==['getMultirotorState']


@pytest.mark.parametrize('failure',[False,True])
def test_control_samples_flush_after_stop_on_success_and_execution_failure(tmp_path,failure):
    workflow,stage,vehicle,planner,trace,clock=continuous_setup(target_x=14)
    trace=VisualGoalTrace(tmp_path)
    workflow.trace_factory=lambda:trace
    owner_write=trace.control_samples
    writes=[]
    def write(samples,**kwargs):
        assert vehicle.hovers > 0  # Synthetic stop occurred before CSV I/O.
        writes.append(len(samples))
        owner_write(samples,**kwargs)
    trace.control_samples=write
    if failure:
        move=vehicle.stream_velocity
        def fail_after_commands(*args):
            if len(vehicle.commands)>=5:raise RuntimeError('synthetic transport failure')
            return move(*args)
        vehicle.stream_velocity=fail_after_commands
    try:
        result=workflow.run(stage)
        assert result.success is not failure
        assert writes and writes[0]>0
        assert not workflow.continuous.stream.telemetry
        with (trace.path/'control_samples.csv').open(encoding='utf-8',newline='') as stream:
            rows=list(csv.DictReader(stream))
        assert len(rows)==writes[0]
        assert 'pitch_deg' in rows[0] and 'yaw_error_deg' in rows[0]
        assert 'commanded_velocity_norm' in rows[0]
        assert float(rows[0]['dt_s'])>0
        assert rows[0]['pitch_deg']==''  # Fake does not invent attitude measurements.
    finally:
        workflow.close()
