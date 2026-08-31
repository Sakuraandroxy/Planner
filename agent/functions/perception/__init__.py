"""Perception functions."""

from agent.functions.perception.bearing_tracker import (
    TargetBearingObservation,
    TargetBearingTracker,
    bbox_center_angle_deg,
    detection_is_excluded_by_bearing,
    metric_depth_usable,
)
from agent.functions.perception.roof_plane import RoofPlaneEstimate, estimate_down_roof_plane
from agent.functions.perception.camera_geometry import (
    GeometryShadowTracker,
    attach_detection_camera_context,
    camera_to_world,
    depth_map_to_world_points,
    depth_map_to_world_samples,
    navigation_to_world,
    pixel_depth_to_world,
    pixel_to_world_ray,
    project_world_to_pixel,
    triangulate_world_rays,
    world_to_camera,
    world_to_navigation,
)

__all__ = [
    "TargetBearingObservation",
    "TargetBearingTracker",
    "bbox_center_angle_deg",
    "detection_is_excluded_by_bearing",
    "metric_depth_usable",
    "RoofPlaneEstimate",
    "estimate_down_roof_plane",
    "GeometryShadowTracker",
    "attach_detection_camera_context",
    "camera_to_world",
    "depth_map_to_world_points",
    "depth_map_to_world_samples",
    "navigation_to_world",
    "pixel_depth_to_world",
    "pixel_to_world_ray",
    "project_world_to_pixel",
    "triangulate_world_rays",
    "world_to_camera",
    "world_to_navigation",
]
