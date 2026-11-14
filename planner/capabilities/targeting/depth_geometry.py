"""AirSim camera basis: +X optical forward, +Y image right, +Z image down.

DepthPerspective is ray range (AirSim Image APIs documentation). Calibration
must explicitly confirm the individual RGB/depth FOV and registered camera.
"""
import math
import numpy as np

from planner.domain.observation import Observation
from planner.domain.visual_goal import GoalGeometry, GoalObservation, SurfacePointEstimate
from .depth_surface_sampling import sample_surface_depth
from .depth_foreground import DepthForegroundFilter


class DepthGeometry:
    def __init__(self, minimum_m: float, verified: bool, *, foreground_filter: DepthForegroundFilter | None = None):
        self.minimum_m, self.verified = minimum_m, verified
        self.foreground_filter = foreground_filter

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
        model_points = goal.surface_points is not None
        source = "model_surface_point_depth" if model_points else "bbox_center_depth"
        try:
            depth = self.validate(obs)
            # Explicit empty model points never fall back to a generated center.
            points = goal.surface_points if model_points else (
                (goal.surface_point,) if goal.surface_point is not None else ())
            if not points or goal.bbox is None:
                return GoalGeometry(False, "target supplies no visible surface points", position_source=source)
            samples = tuple(self._sample_point(obs, goal.bbox, depth, point) for point in points)
            depth_filter = None
            if self.foreground_filter is not None:
                pixels = tuple(self.project_camera(obs, self.ray(obs, point)) for point in points)
                samples, depth_filter = self.foreground_filter.filter(
                    obs, goal.bbox, depth, self.minimum_m, samples, pixels)
            valid = [sample for sample in samples if sample.valid]
            if not valid:
                return GoalGeometry(False, "; ".join(sample.reason for sample in samples),
                    surface_samples=samples, position_source=source, depth_filter=depth_filter)
            chosen = min(valid, key=lambda sample: sample.distance_m)
            return GoalGeometry(True, chosen.reason, chosen.distance_m, chosen.world_point,
                surface_samples=samples, position_source=source, depth_filter=depth_filter)
        except (ValueError, TypeError, IndexError) as exc:
            return GoalGeometry(False, str(exc), position_source=source)

    def _sample_point(self, obs, bbox, depth, point) -> SurfacePointEstimate:
        ray = self.ray(obs, point)
        pixel = self.project_camera(obs, ray)
        sample = sample_surface_depth(depth, pixel, bbox, obs.intrinsics,
            obs.depth_intrinsics, self.minimum_m)
        if not sample.valid:
            return SurfacePointEstimate(point, False, sample.reason)
        world = np.asarray(obs.camera_position_world) + np.asarray(obs.rotation_camera_to_world) @ (ray*sample.depth_m)
        vehicle = obs.vehicle_pose
        distance = math.dist(world, (vehicle.x, vehicle.y, vehicle.z))
        return SurfacePointEstimate(point, True, sample.reason, sample.depth_m,
            tuple(float(v) for v in world), distance)
