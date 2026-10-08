"""Static wall and deterministic clock; no RPCs or model requests."""
import math

import numpy as np
from PIL import Image

from planner.adapters.airsim.camera_model import intrinsics_from_horizontal_fov
from planner.domain.observation import Observation
from planner.domain.pose import RelativePoseDelta, WorldPose
from planner.domain.trajectory import RelativeTrajectory
from planner.domain.visual_goal import GoalObservation, GoalVisibility


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def wall_observation(distance=4.0, pose=None, size=(128, 128), depth_size=None, clock=None, stamp=1):
    pose = pose or WorldPose(0, 0, -4, 0)
    clock = clock or Clock()
    width, height = depth_size or size
    k = intrinsics_from_horizontal_fov(width, height, 90)
    ys, xs = np.mgrid[:height, :width]
    norms = np.sqrt(1+((xs-k.cx)/k.fx)**2+((ys-k.cy)/k.fy)**2)
    image = Image.new("RGB", size, "red")
    depth = distance*norms  # Ray-range to a camera-plane-parallel wall.
    yaw = math.radians(pose.yaw_deg)
    rotation = ((math.cos(yaw), -math.sin(yaw), 0), (math.sin(yaw), math.cos(yaw), 0), (0, 0, 1))
    return Observation(image, depth, Image.new("RGB", (width, height)), pose, "front_center",
                       intrinsics_from_horizontal_fov(*size, 90), (pose.x, pose.y, pose.z), rotation,
                       stamp, k, "ray_range", clock(), f"capture-{stamp}", stamp, True)


VISIBLE = GoalObservation(GoalVisibility.VISIBLE, (0.25, 0.25, 0.75, 0.75), (0.5, 0.5),
                          "red house", "red facade is visible", True)
INVISIBLE = GoalObservation(GoalVisibility.NOT_VISIBLE, None, None, "red house", "no target", True)


class WallSource:
    def __init__(self, vehicle, clock, target_x=6.0):
        self.vehicle, self.clock, self.target_x = vehicle, clock, target_x
        self.captures = 0
        self.replay = False

    def capture(self, *, budget=None):
        self.captures += 1
        self.clock.sleep(0.01)
        return wall_observation(self.target_x-self.vehicle.pose.x, self.vehicle.pose,
                                clock=self.clock, stamp=1 if self.replay else self.captures)


class StaticObserver:
    def __init__(self, goal=VISIBLE):
        self.goal, self.calls = goal, []

    def observe(self, observation, description, reference=None, **kwargs):
        self.calls.append((observation, description, reference, kwargs))
        return self.goal


class ForwardPlanner:
    def __init__(self, points=None):
        self.points = points or tuple(RelativePoseDelta(2, 0, 0, 0) for _ in range(5))
        self.calls = []

    def plan(self, observation, instruction, **kwargs):
        self.calls.append((observation, instruction, kwargs))
        return RelativeTrajectory(self.points)
