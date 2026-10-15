"""AirSim camera basis: +X optical forward, +Y image right, +Z image down.

DepthPerspective is ray range (AirSim Image APIs documentation). Calibration
must explicitly confirm the individual RGB/depth FOV and registered camera.
"""
import math
import numpy as np

from planner.domain.observation import Observation
from planner.domain.visual_goal import GoalGeometry, GoalObservation
from .depth_surface_sampling import sample_surface_depth


class DepthGeometry:
    def __init__(self, minimum_m: float, maximum_m: float, verified: bool):
        self.minimum_m, self.maximum_m, self.verified = minimum_m, maximum_m, verified

    def validate(self, obs: Observation):
        if not self.verified or obs.depth_kind != "ray_range" or obs.depth_intrinsics is None:
            raise ValueError("RGB/depth calibration and ray-range semantics must be verified")
        depth = np.asarray(obs.depth_meters)
        k = obs.depth_intrinsics
        if depth.shape != (k.height, k.width) or obs.rgb.size != (obs.intrinsics.width, obs.intrinsics.height):
            raise ValueError("image dimensions do not match calibrated intrinsics")
        for intrinsics in (obs.intrinsics, k):
            if not all(math.isfinite(v) and v > 0 for v in (intrinsics.fx, intrinsics.fy)):
                raise ValueError("invalid camera focal lengths")
            if intrinsics.width <= 0 or intrinsics.height <= 0 or not all(
                math.isfinite(v) for v in (intrinsics.cx, intrinsics.cy)
            ):
                raise ValueError("invalid camera image dimensions or principal point")
        rotation = np.asarray(obs.rotation_camera_to_world, dtype=float)
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all() or not np.allclose(
            rotation.T @ rotation, np.eye(3), atol=1e-3
        ) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3):
            raise ValueError("invalid camera rotation")
        if not np.isfinite(obs.camera_position_world).all():
            raise ValueError("invalid camera origin")
        return depth

    @staticmethod
    def ray(obs, point):
        k = obs.intrinsics
        direction = np.array((1.0, (point[0]*k.width-k.cx)/k.fx, (point[1]*k.height-k.cy)/k.fy))
        return direction / np.linalg.norm(direction)

    @staticmethod
    def project_camera(obs, camera_point):
        if camera_point[0] <= 0:
            return None
        k = obs.depth_intrinsics
        return (k.fx*camera_point[1]/camera_point[0]+k.cx,
                k.fy*camera_point[2]/camera_point[0]+k.cy)

    def estimate(self, obs: Observation, goal: GoalObservation):
        try:
            depth = self.validate(obs)
            if goal.surface_point is None or goal.bbox is None:
                return GoalGeometry(False, "target has no visible surface")
            ray = self.ray(obs, goal.surface_point)
            pixel = self.project_camera(obs, ray)
            sample = sample_surface_depth(depth, pixel, goal.bbox, obs.intrinsics,
                obs.depth_intrinsics, self.minimum_m, self.maximum_m)
            if not sample.valid:
                return GoalGeometry(False, sample.reason)
            world = np.asarray(obs.camera_position_world) + np.asarray(obs.rotation_camera_to_world) @ (ray*sample.depth_m)
            vehicle = obs.vehicle_pose
            distance = math.dist(world, (vehicle.x, vehicle.y, vehicle.z))
            return GoalGeometry(True, sample.reason, distance,
                                tuple(float(v) for v in world))
        except (ValueError, TypeError, IndexError) as exc:
            return GoalGeometry(False, str(exc))
