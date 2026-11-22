"""深度组入口：扇面测距、像素深度读数与目标几何估计（裁决 R24）。"""

from ..free_space import FreeSpaceProfile, FreeSpaceProfileSettings, measure_free_space
from ..targeting.depth_foreground import DepthForegroundFilter
from ..targeting.depth_geometry import DepthGeometry
from ..targeting.depth_surface_sampling import sample_surface_depth

__all__ = [
    "DepthForegroundFilter",
    "DepthGeometry",
    "FreeSpaceProfile",
    "FreeSpaceProfileSettings",
    "measure_free_space",
    "sample_surface_depth",
]
