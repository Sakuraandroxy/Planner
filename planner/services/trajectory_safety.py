"""Conservative observed-space checks. Unknown pixels never certify free space.

This is a sampled static-scene corridor check, not a complete obstacle planner.
Previously certified volumes and an explicitly confirmed initial free bubble
support the part of the robot behind its single forward camera.
"""
from dataclasses import dataclass
import itertools
import math

import numpy as np

from planner.domain.trajectory import MotionTrajectory
from planner.domain.pose import wrap_yaw_deg


@dataclass(frozen=True)
class SafetySelection:
    motion: MotionTrajectory
    reason: str
    certified_centers: tuple[tuple[float, float, float], ...] = ()


class TrajectorySafetyChecker:
    def __init__(self, geometry, limits, motion_limits, initial_clearance_confirmed: bool):
        self.geometry, self.limits, self.motion_limits = geometry, limits, motion_limits
        self.initial_clearance_confirmed = initial_clearance_confirmed

    def begin(self, observation):
        self.geometry.validate(observation)
        if not self.initial_clearance_confirmed:
            raise ValueError("initial surrounding clearance must be confirmed for single-view operation")
        pose = observation.vehicle_pose
        return [((pose.x, pose.y, pose.z), self.limits.initial_clearance_radius_m)]

    def clearance(self, observation):
        cloud = self.geometry.point_cloud(observation)
        if cloud.size == 0:
            return False
        vehicle = observation.vehicle_pose
        camera = np.asarray(observation.camera_position_world)
        rotation = np.asarray(observation.rotation_camera_to_world)
        points = cloud @ rotation.T + camera
        return bool(np.min(np.linalg.norm(points-(vehicle.x, vehicle.y, vehicle.z), axis=1)) >
                    self.limits.clearance_radius_m)

    def _supported(self, point, obs, depth, free_balls):
        if any(math.dist(point, center) <= radius for center, radius in free_balls):
            return True
        camera = np.asarray(obs.rotation_camera_to_world).T @ (np.asarray(point)-obs.camera_position_world)
        pixel = self.geometry.project_camera(obs, camera)
        if pixel is None:
            return False
        x, y = (int(round(v)) for v in pixel)
        height, width = depth.shape
        if not (1 <= x < width-1 and 1 <= y < height-1):
            return False
        patch = depth[y-1:y+2, x-1:x+2]
        # Include a pixel-neighborhood margin rather than relying on one range.
        return bool(self.geometry.valid(patch).all() and
                    np.linalg.norm(camera)+self.limits.tracking_margin_m < np.min(patch))

    def _sphere_safe(self, center, obs, depth, free_balls, cloud_world):
        radius = self.limits.clearance_radius_m
        if cloud_world.size and np.min(np.linalg.norm(cloud_world-center, axis=1)) <= radius:
            return False
        # Sample center and cube covering the required swept body radius.
        offsets = [(0, 0, 0)] + list(itertools.product((-radius, 0, radius), repeat=3))
        return all(self._supported(np.asarray(center)+offset, obs, depth, free_balls) for offset in offsets)

    def select(self, motion, observation, initial_pose, free_balls, target_point, lower_standoff, *,
               additional_standoff_points: tuple[tuple[float, float, float], ...] = ()):
        depth = self.geometry.validate(observation)
        cloud_world = (self.geometry.point_cloud(observation) @
                       np.asarray(observation.rotation_camera_to_world).T + observation.camera_position_world)
        selected, certified = [], []
        distance_sum = time_sum = 0.0
        reason = "complete predicted trajectory supported"
        start_yaw = observation.vehicle_pose.yaw_deg
        for segment in motion.segments:
            a, b = segment.start, segment.end
            av, bv = np.array((a.x, a.y, a.z)), np.array((b.x, b.y, b.z))
            distance = float(np.linalg.norm(bv-av))
            duration = (segment.duration_s or math.inf)+self.motion_limits.settle_time_s+self.limits.execution_overhead_s
            if distance_sum+distance > self.limits.maximum_round_distance_m or time_sum+duration > self.limits.maximum_round_motion_s:
                reason = "complete segment exceeds execution distance/time horizon"
                break
            if abs(b.z-initial_pose.z) > self.limits.maximum_vertical_offset_m:
                reason = "segment violates task height envelope"
                break
            angle = abs(wrap_yaw_deg(b.yaw_deg-start_yaw))
            if distance > 1e-6 and angle >= self.limits.reobserve_heading_change_deg:
                reason = "turn with translation needs a new view; request pure yaw first"
                break
            centers = tuple(tuple(av+(bv-av)*fraction) for fraction in np.linspace(
                0, 1, max(2, math.ceil(distance/self.limits.safety_sample_spacing_m)+1)
            ))
            if any(math.dist(p, surface) < lower_standoff for p in centers
                   for surface in (target_point, *additional_standoff_points)):
                reason = "segment overshoots target standoff band"
                break
            if not all(self._sphere_safe(p, observation, depth, free_balls, cloud_world) for p in centers):
                reason = "obstacle or unobserved swept volume"
                break
            selected.append(segment)
            certified.extend(centers)
            distance_sum += distance
            time_sum += duration
            if angle >= self.limits.reobserve_heading_change_deg:
                reason = "heading change requires reobservation before next segment"
                break
        return SafetySelection(MotionTrajectory(tuple(selected)), reason, tuple(certified))

    def record_completed(self, free_balls, selection):
        # Task-local static free-space certificates, not a persistent mapping subsystem.
        return (free_balls+[(center, self.limits.clearance_radius_m)
                            for center in selection.certified_centers])[-2000:]
