"""Single attached camera control; no workflow decisions or flight commands."""
import math
import numpy as np

from planner.domain.visual_goal_above import CameraMountPose
from .camera_model import quaternion_to_rotation


def vector(v):
    return np.array((v.x_val,v.y_val,v.z_val),dtype=float)


def quaternion(q):
    return np.array((q.x_val,q.y_val,q.z_val,q.w_val),dtype=float)


def rotation(q):
    return np.asarray(quaternion_to_rotation(*q))


def matrix_quaternion(matrix):
    # Symmetric eigenproblem avoids singular Euler angles at a downward view.
    m = np.asarray(matrix)
    k = np.array(((m[0,0]-m[1,1]-m[2,2],m[1,0]+m[0,1],m[2,0]+m[0,2],m[2,1]-m[1,2]),
                  (m[1,0]+m[0,1],m[1,1]-m[0,0]-m[2,2],m[2,1]+m[1,2],m[0,2]-m[2,0]),
                  (m[2,0]+m[0,2],m[2,1]+m[1,2],m[2,2]-m[0,0]-m[1,1],m[1,0]-m[0,1]),
                  (m[2,1]-m[1,2],m[0,2]-m[2,0],m[1,0]-m[0,1],np.trace(m))))/3
    _, eigenvectors = np.linalg.eigh(k)
    q = eigenvectors[:,-1]
    return q if q[3] >= 0 else -q


def interpolate_body(before, after, timestamp_ns):
    """Image-time body pose, independent of camera/gimbal orientation."""
    span = int(after.timestamp)-int(before.timestamp)
    t = min(1.,max(0.,(timestamp_ns-int(before.timestamp))/span)) if span > 0 else 0.5
    a,b = before.kinematics_estimated,after.kinematics_estimated
    qa,qb = quaternion(a.orientation),quaternion(b.orientation)
    if qa@qb < 0:
        qb = -qb
    dot = float(np.clip(qa@qb,-1,1))
    if dot > .9995:
        q = (1-t)*qa+t*qb
    else:
        angle = math.acos(dot)
        q = (math.sin((1-t)*angle)*qa+math.sin(t*angle)*qb)/math.sin(angle)
    q /= np.linalg.norm(q)
    return (1-t)*vector(a.position)+t*vector(b.position),rotation(q)


class AirSimCameraView:
    def __init__(self, connection, camera_id):
        self.connection, self.camera_id = connection,camera_id
        self.position = None

    def read_mount(self):
        # Read while held, for the stage original or the last visible mount.
        body = self.connection.call("getMultirotorState").kinematics_estimated
        camera = self.connection.call("simGetCameraInfo",self.camera_id).pose
        body_rotation = rotation(quaternion(body.orientation))
        position = body_rotation.T @ (vector(camera.position)-vector(body.position))
        relative_rotation = body_rotation.T @ rotation(quaternion(camera.orientation))
        self.position = tuple(map(float,position))
        return CameraMountPose(self.position,tuple(tuple(map(float,row)) for row in relative_rotation))

    def point(self, pitch_deg, yaw_deg=0.0):
        import airsim
        if self.position is None:
            raise RuntimeError("save the original camera mount before adjusting it")
        pose = airsim.Pose(airsim.Vector3r(*self.position),
                          airsim.to_quaternion(math.radians(pitch_deg),0.,math.radians(yaw_deg)))
        self.connection.call("simSetCameraPose",self.camera_id,pose)

    def point_at_world(self, point):
        if self.position is None:
            raise RuntimeError("save the original camera mount before adjusting it")
        body = self.connection.call("getMultirotorState").kinematics_estimated
        body_rotation = rotation(quaternion(body.orientation))
        camera_origin = vector(body.position)+body_rotation@np.asarray(self.position)
        direction = body_rotation.T@(np.asarray(point)-camera_origin)
        self._point_direction(direction)

    def point_down(self):
        body = self.connection.call("getMultirotorState").kinematics_estimated
        body_rotation = rotation(quaternion(body.orientation))
        self._point_direction(body_rotation.T@np.array((0.,0.,1.)))

    def point_direction_world(self, direction):
        body = self.connection.call("getMultirotorState").kinematics_estimated
        self._point_direction(rotation(quaternion(body.orientation)).T@np.asarray(direction))

    def _point_direction(self, direction):
        horizontal = math.hypot(direction[0],direction[1])
        if math.hypot(*direction) < 1e-9:
            return
        pitch = math.degrees(math.atan2(-direction[2],horizontal))
        yaw = math.degrees(math.atan2(direction[1],direction[0]))
        self.point(pitch,yaw)

    def restore(self, mount):
        import airsim
        q = matrix_quaternion(mount.rotation)
        self.connection.call("simSetCameraPose",self.camera_id,
            airsim.Pose(airsim.Vector3r(*mount.position),airsim.Quaternionr(*map(float,q))))
        self.position = mount.position
