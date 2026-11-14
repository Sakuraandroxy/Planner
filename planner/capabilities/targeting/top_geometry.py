"""Roof contours and sparse metric supports produce finite planar polygons."""
import math
import numpy as np

from planner.domain.visual_goal import GoalObservation, GoalVisibility
from .depth_geometry import DepthGeometry
from planner.domain.above_polygon import in_ring
from .top_plane import reconstruct_region


def contains_xy(region, x, y):
    return region.height_z(x, y) is not None


def top_height(regions, x, y):
    # New observation first; older partial views supply only missing coverage.
    captures = dict.fromkeys(r.capture_id for r in regions)
    for capture in captures:
        heights = [z for r in regions if r.capture_id == capture and (z := r.height_z(x,y)) is not None]
        if heights:
            return min(heights)
        if any(in_ring(x,y,r.outline_world) for r in regions if r.capture_id == capture):
            return None  # A newly observed hole must not be filled by an older footprint.
    return None


class TopGeometry:
    def __init__(self, minimum_m=0.5):
        self.geometry = DepthGeometry(minimum_m, True)

    def surface_points(self, obs, detection):
        if detection.bbox is None:
            return ()
        goal = GoalObservation(GoalVisibility.VISIBLE,detection.bbox.coordinates,None,"target","model surfaces",
                               surface_points=detection.surface_points)
        result = self.geometry.estimate(obs,goal)
        return tuple(s.world_point for s in result.surface_samples if s.valid)

    def estimate(self, obs, detection):
        depth = self.geometry.validate(obs)
        result = []
        for region in detection.top_regions:
            supports,pixels = [],[]
            for point in region.support_points:
                if not in_ring(*point, region.outline) or any(in_ring(*point, h) for h in region.holes):
                    continue
                ray = self.geometry.ray(obs, point)
                x, y = self.geometry.project_camera(obs, ray)
                x, y = int(round(x)), int(round(y))
                if not (0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]):
                    continue
                distance = float(depth[y, x])
                if not math.isfinite(distance) or distance < self.geometry.minimum_m or distance <= 0:
                    continue
                # The sampled range belongs to the rounded depth pixel's ray.
                k = obs.depth_intrinsics
                depth_ray = np.array((1.,(x-k.cx)/k.fx,(y-k.cy)/k.fy))
                depth_ray /= np.linalg.norm(depth_ray)
                supports.append(np.asarray(obs.camera_position_world)+np.asarray(obs.rotation_camera_to_world)@(depth_ray*distance))
                pixels.append(point)
            patch = reconstruct_region(obs,region,pixels,supports)
            if patch is not None:
                result.append(patch)
        return tuple(result)

def choose_destination(regions, pose, height, *, minimum=False):
    from planner.domain.pose import WorldPose
    candidates = []
    for region in regions:
        z = region.height_z(pose.x,pose.y)
        if z is not None:
            candidates.append((pose.x,pose.y,z))
        for x,y,_ in region.support_points:
            z = region.height_z(x,y)
            if z is not None:
                candidates.append((x,y,z))
        # A centroid is only a candidate: concavities and holes can exclude it.
        x,y = (sum(p[a] for p in region.outline_world)/len(region.outline_world) for a in (0,1))
        z = region.height_z(x,y)
        if z is not None:
            candidates.append((x,y,z))
    if not candidates:
        return None
    point = min(candidates, key=lambda p: math.hypot(p[0]-pose.x,p[1]-pose.y))
    z = point[2]-height
    return WorldPose(point[0], point[1], min(pose.z,z) if minimum else z, pose.yaw_deg)


def region_summary(region):
    """The finite observed polygon and fitted plane, not a complete building map."""
    return {"capture_id": region.capture_id, "support_points_world": region.support_points,
            "outline_world":region.outline_world,"holes_world":region.holes_world,
            "plane":region.plane,
            "representation":"observed roof polygon with holes and a local fitted plane"}
