"""Use a box edge as a viewing cue, never as a measured roof surface."""
from dataclasses import dataclass
import math

import numpy as np

from planner.domain.observation import Observation
from planner.domain.target_detection import NormalizedBoundingBox


@dataclass(frozen=True)
class BboxTopView:
    direction_world: tuple[float, float, float]
    edge_elevations_deg: tuple[float, ...]
    clipped_top: bool
    ascend: bool

    @property
    def top_observation_possible(self) -> bool:
        return not self.clipped_top and max(self.edge_elevations_deg) < 0.


def bbox_top_view(frame: Observation, bbox: NormalizedBoundingBox) -> BboxTopView:
    k = frame.intrinsics
    rotation = np.asarray(frame.rotation_camera_to_world)

    def ray(u, v):
        direction = rotation @ np.array((1., (u*k.width-k.cx)/k.fx,
                                         (v*k.height-k.cy)/k.fy))
        return direction/np.linalg.norm(direction)

    # A rolled camera has a sloping horizon. Compare several edge rays with
    # world-horizontal; the image midpoint is not a horizon measurement.
    us = (bbox.xmin, (bbox.xmin+bbox.xmax)/2, bbox.xmax)
    rays = [ray(u, bbox.ymin) for u in us]
    elevations = tuple(math.degrees(math.atan2(-r[2], math.hypot(r[0], r[1]))) for r in rays)
    ascend = max(elevations) >= 0.
    clipped = bbox.ymin*k.height <= .5
    # When above the apparent top, aim into the target's upper portion, not
    # vertically below the UAV. At a cropped edge first bring the edge in view.
    v = bbox.ymin if ascend or clipped else bbox.ymin+(bbox.ymax-bbox.ymin)/4
    direction = ray(us[1], v)
    return BboxTopView(tuple(map(float, direction)), elevations, clipped, ascend)
