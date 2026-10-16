"""Real shared follower with synthetic dynamics; no network or flight hardware."""
import math
from dataclasses import replace

from planner.domain.motion import MotionLimits
from planner.domain.pose import WorldPose
from planner.domain.trajectory import WorldTrajectory
from planner.services.continuous_execution import ContinuousExecution
from planner.services.execution import ExecutionService
from planner.services.trajectory_queue import WorldWaypointQueue
from planner.services.motion.continuous_reference import distance_to_segment, xyz
from tests.fakes.visual_goal import Clock
from tests.integration.test_continuous_visual_goal import StreamVehicle


class LagVehicle(StreamVehicle):
    """Additional 0.2s velocity lag beyond the reference generator response."""

    def stream_velocity(self, goal, rate, duration):
        self.commands.append((self.clock(),self.pose,goal,rate))
        alpha = 1-math.exp(-duration/.2)
        velocity = tuple(v+(g-v)*alpha for v,g in zip(self.velocity,goal))
        delta = tuple((a+b)*.5*duration for a,b in zip(self.velocity,velocity))
        self.pose = replace(self.pose,x=self.pose.x+delta[0],y=self.pose.y+delta[1],
            z=self.pose.z+delta[2],yaw_deg=(self.pose.yaw_deg+rate*duration+180)%360-180)
        self.velocity = velocity
        self.clock.sleep(duration)


def follow(points, yaw_deltas, vehicle_type=StreamVehicle):
    clock = Clock()
    vehicle = vehicle_type(clock)
    queue = WorldWaypointQueue(.3, 2)
    original = WorldTrajectory(vehicle.pose, tuple(points), tuple(yaw_deltas))
    queue.load_prefix(original)
    stream = ContinuousExecution(ExecutionService(vehicle, clock),vehicle,MotionLimits(),2,clock)
    stream.begin(queue,1)
    passes, positions = [], []
    for _ in range(2400):
        ids = stream.tick()
        positions.append(xyz(vehicle.pose))
        if ids:
            passes.append((ids, math.hypot(*vehicle.velocity)))
        if not queue.items:
            break
    assert not queue.items
    return stream, vehicle, passes, positions


def test_short_segments_with_large_yaw_pass_without_repeated_brake_to_hover():
    points = [WorldPose(.7*(i+1),0,0,yaw) for i,yaw in enumerate([20,35,45,50,50])]
    stream, vehicle, passes, _ = follow(points,(20,15,10,5,0))
    # The old implementation dipped below .1m/s between these intermediate
    # points. Start/end stops remain intentional; no forced cruise-speed floor.
    assert all(speed > .2 for _,speed in passes[:-1])
    assert len(passes) == 5
    assert abs(vehicle.pose.yaw_deg-50) <= 2
    assert sum(sample.aligning for sample in stream.telemetry) <= 3
    commands = vehicle.commands
    velocities = [(0.,0.,0.)]+[row[2] for row in commands]
    accelerations = [tuple((b-a)/.05 for a,b in zip(first,second))
                     for first,second in zip(velocities,velocities[1:])]
    assert all(math.hypot(*a) <= 1.5+1e-8 for a in accelerations)
    assert all(math.dist(a,b) <= 3*.05+1e-8 for a,b in zip(accelerations,accelerations[1:]))
    rates = [0.]+[row[3] for row in commands]
    yaw_accelerations = [(b-a)/.05 for a,b in zip(rates,rates[1:])]
    assert all(abs(rate) <= 30+1e-8 for rate in rates)
    assert all(abs(a) <= 30+1e-8 for a in yaw_accelerations)
    assert all(abs(b-a) <= 60*.05+1e-8 for a,b in zip(yaw_accelerations,yaw_accelerations[1:]))


def test_right_angle_preview_stays_within_existing_path_tolerance():
    points = [WorldPose(4,0,0,0),WorldPose(4,4,0,0),WorldPose(4,8,0,0)]
    _, vehicle, passes, positions = follow(points,(0,0,0))
    segments = list(zip([(0,0,0)]+[xyz(p) for p in points[:-1]], [xyz(p) for p in points]))
    assert max(min(distance_to_segment(p,a,b) for a,b in segments) for p in positions) <= .3
    assert passes[0][1] > .15
    assert all(abs(command[3]) < 1e-9 for command in vehicle.commands)  # Yaw is not velocity heading.


def test_queue_tail_brakes_to_low_measured_speed_before_hover():
    stream, vehicle, _, _ = follow([WorldPose(5,0,0,0)],(0,))
    assert abs(vehicle.pose.x-5) <= .3
    assert math.hypot(*stream.state.velocity_world) <= MotionLimits().stopped_speed_mps
    assert vehicle.hovers == 1


def test_corner_exit_does_not_accelerate_early_with_additional_vehicle_lag():
    points = [WorldPose(4,0,0,0),WorldPose(4,4,0,0),WorldPose(4,8,0,0)]
    _, _, passes, positions = follow(points,(0,0,0),LagVehicle)
    segments = list(zip([(0,0,0)]+[xyz(p) for p in points[:-1]], [xyz(p) for p in points]))
    assert max(min(distance_to_segment(p,a,b) for a,b in segments) for p in positions) <= .3
    assert passes[0][1] > .15


def test_rotation_barrier_finishes_signed_turn_before_departing():
    points = [WorldPose(4,0,0,0),WorldPose(4,0,0,90),WorldPose(4,4,0,90)]
    _, vehicle, _, _ = follow(points,(0,90,0))
    turning = [row for row in vehicle.commands if abs(row[1].yaw_deg-90) > 2 and row[1].x > 3.7]
    assert turning and max(abs(row[1].y) for row in turning) < .15
    assert abs(vehicle.pose.yaw_deg-90) <= 2


def test_zero_motion_padding_does_not_create_an_in_place_turn_or_dwell():
    points = [WorldPose(4,0,0,0),WorldPose(4,0,0,0),WorldPose(8,0,0,0)]
    _, _, passes, _ = follow(points,(0,0,0))
    assert passes[0][0] == (1,2)
    assert passes[0][1] > 1.5


def test_online_append_keeps_command_velocity_and_acceleration_continuous():
    clock = Clock()
    vehicle = StreamVehicle(clock)
    queue = WorldWaypointQueue(.3,2)
    poses = tuple(WorldPose(2*(i+1),0,0,0) for i in range(5))
    queue.load_batch(WorldTrajectory(vehicle.pose,poses,(0,)*5))
    stream = ContinuousExecution(ExecutionService(vehicle,clock),vehicle,MotionLimits(),2,clock)
    stream.begin(queue,1)
    appended = False
    for _ in range(1200):
        if not appended and vehicle.pose.x > 3:
            velocity = stream.follower.velocity
            acceleration = stream.follower.translation.acceleration
            tail = queue.items[-1].end
            world = WorldTrajectory(tail,tuple(WorldPose(12+2*i,0,0,0) for i in range(5)),(0,)*5)
            queue.append_continuation(world,tuple(p.waypoint_id for p in queue.items),queue.revision,vehicle.pose)
            stream.tick()
            assert math.dist(velocity,stream.follower.velocity) <= 1.5*.05+1e-8
            assert math.dist(acceleration,stream.follower.translation.acceleration) <= 3*.05+1e-8
            appended = True
        else:
            stream.tick()
        if not queue.items:
            break
    assert appended and not queue.items
    assert vehicle.hovers == 1  # Queue tail only, no stop at the batch boundary.
