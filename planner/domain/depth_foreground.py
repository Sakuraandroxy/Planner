"""Settings and evidence for local depth-layer filtering, not object recognition."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DepthForegroundSettings:
    enabled: bool = True
    pixel_radius: int = 3
    depth_gap_ratio: float = 4.0

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("depth_foreground_filter.enabled must be boolean")
        if type(self.pixel_radius) is not int or not 1 <= self.pixel_radius <= 32:
            raise ValueError('depth_foreground_filter.pixel_radius must be an integer from 1 to 32')
        if (isinstance(self.depth_gap_ratio, bool) or not isinstance(self.depth_gap_ratio,(int,float))
                or not math.isfinite(self.depth_gap_ratio) or self.depth_gap_ratio <= 1):
            raise ValueError('depth_foreground_filter.depth_gap_ratio must be finite and greater than 1')


@dataclass(frozen=True)
class DepthPointLayer:
    image_point: tuple[float, float]
    near_forward_min_m: float
    near_forward_max_m: float
    far_forward_min_m: float
    cutoff_forward_m: float


@dataclass(frozen=True)
class DepthForegroundResult:
    applied: bool
    reason: str
    rejected_points: int = 0
    point_layers: tuple[DepthPointLayer, ...] = ()
