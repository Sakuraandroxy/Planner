"""Task-only wrapper: image-time body pose must not be inferred from a fixed mount."""
from dataclasses import replace
import math

from planner.domain.pose import WorldPose
from .above_camera import interpolate_body


class AboveObservationSource:
    def __init__(self, source, read_connection):
        self.source, self.connection = source,read_connection

    def capture_moving(self, *, budget=None):
        before = self.connection.call("getMultirotorState")
        frame = self.source.capture_moving(budget=budget)
        after = self.connection.call("getMultirotorState")
        position, rotation = interpolate_body(before,after,frame.timestamp_ns)
        yaw = math.degrees(math.atan2(rotation[1,0],rotation[0,0]))
        return replace(frame,vehicle_pose=WorldPose(*map(float,position),yaw))

    def close(self):
        self.source.close()
