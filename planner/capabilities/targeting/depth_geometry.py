"""AirSim camera basis: +X optical forward, +Y image right, +Z image down.

DepthPerspective is ray range (AirSim Image APIs documentation). Calibration
must explicitly confirm the individual RGB/depth FOV and registered camera.
"""
import math
import numpy as np

from planner.domain.observation import Observation
from planner.domain.visual_goal import GoalGeometry, GoalObservation
from planner.domain.visual_goal_limits import VisualGoalLimits
from planner.domain.target_surface import GoalSurfaceObservation, SurfaceEstimate


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

    def valid(self, values):
        return np.isfinite(values) & (values >= self.minimum_m) & (values <= self.maximum_m)

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

    def estimate(self, obs: Observation, goal: GoalObservation, limits: VisualGoalLimits):
        try:
            depth = self.validate(obs)
            if goal.surface_point is None or goal.bbox is None:
                return GoalGeometry(False, "target has no visible surface")
            ray = self.ray(obs, goal.surface_point)
            pixel = self.project_camera(obs, ray)
            k = obs.depth_intrinsics
            u, v = pixel
            if not (0 <= u < k.width and 0 <= v < k.height):
                return GoalGeometry(False, "target ray is outside depth coverage")
            half_x = max(1, math.ceil(limits.depth_window_fraction*k.width/2))
            half_y = max(1, math.ceil(limits.depth_window_fraction*k.height/2))
            x0, x1 = max(0, int(u)-half_x), min(k.width, int(u)+half_x+1)
            y0, y1 = max(0, int(v)-half_y), min(k.height, int(v)+half_y+1)
            ys, xs = np.mgrid[y0:y1, x0:x1]
            rgb_k = obs.intrinsics
            rgb_u = ((xs-k.cx)/k.fx*rgb_k.fx+rgb_k.cx)/rgb_k.width
            rgb_v = ((ys-k.cy)/k.fy*rgb_k.fy+rgb_k.cy)/rgb_k.height
            box = goal.bbox
            inside = (rgb_u >= box[0]) & (rgb_u <= box[2]) & (rgb_v >= box[1]) & (rgb_v <= box[3])
            values = depth[y0:y1, x0:x1][inside]
            if values.size < 9:
                return GoalGeometry(False, "too few target-interior depth pixels")
            mask = self.valid(values)
            ratio = float(mask.mean())
            if ratio < limits.minimum_valid_depth_ratio:
                return GoalGeometry(False, "insufficient valid target depth", valid_ratio=ratio)
            median = float(np.median(values[mask]))
            mad = float(np.median(np.abs(values[mask]-median)))
            # Reject mixed foreground/background rather than hiding it by dropping outliers.
            if mad > limits.maximum_depth_mad_m or np.mean(
                np.abs(values[mask]-median) <= max(0.05, limits.maximum_depth_mad_m*2)
            ) < limits.minimum_valid_depth_ratio:
                return GoalGeometry(False, "mixed or unstable target surface depth", valid_ratio=ratio, mad_m=mad)
            world = np.asarray(obs.camera_position_world) + np.asarray(obs.rotation_camera_to_world) @ (ray*median)
            vehicle = obs.vehicle_pose
            distance = math.dist(world, (vehicle.x, vehicle.y, vehicle.z))
            return GoalGeometry(True, "valid target surface depth", distance,
                                tuple(float(v) for v in world), ratio, mad)
        except (ValueError, TypeError, IndexError) as exc:
            return GoalGeometry(False, str(exc))

    def estimate_candidates(self, obs: Observation, goal: GoalSurfaceObservation,
                            limits: VisualGoalLimits) -> tuple[SurfaceEstimate, ...]:
        """Use the same calibrated raw-depth estimator for every candidate."""
        return tuple(SurfaceEstimate(candidate, estimate, estimate.reliable, estimate.reason)
                     for candidate in goal.surface_candidates
                     for estimate in (self.estimate(obs, goal.at(candidate), limits),))

    def point_cloud(self, obs):
        depth = self.validate(obs)
        k = obs.depth_intrinsics
        ys, xs = np.mgrid[:k.height, :k.width]
        rays = np.stack((np.ones(depth.shape), (xs-k.cx)/k.fx, (ys-k.cy)/k.fy), axis=-1)
        rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
        valid = self.valid(depth)
        return rays[valid]*depth[valid][:, None]
