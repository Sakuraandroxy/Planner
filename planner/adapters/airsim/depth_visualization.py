"""Render depth with the selected training protocol, without altering raw meters."""
import math

import numpy as np
from PIL import Image

from planner.domain.depth import (
    FAR_VALID_PIXEL_VALUE, INVALID_DEPTH_SENTINEL_M, LEGACY_DEPTH_MAPPING,
    validate_depth_protocol,
)


def render_depth(depth: np.ndarray, minimum: float, maximum: float,
                 mapping: str = LEGACY_DEPTH_MAPPING) -> Image.Image:
    validate_depth_protocol(mapping, minimum, maximum)
    depth = np.asarray(depth, dtype=np.float32)
    output = np.zeros(depth.shape, dtype=np.uint8)
    if mapping == LEGACY_DEPTH_MAPPING:
        valid = np.isfinite(depth) & (depth >= minimum) & (depth <= maximum)
        if np.any(valid):
            values = (math.log(maximum) - np.log(np.clip(depth[valid], minimum, maximum)))
            values /= math.log(maximum) - math.log(minimum)
            output[valid] = np.clip(values * 255.0, 1, 255).astype(np.uint8)
    else:
        valid = np.isfinite(depth) & (depth > 0) & (depth < INVALID_DEPTH_SENTINEL_M)
        if np.any(valid):
            clipped = np.clip(depth[valid], minimum, maximum)
            fraction = np.log1p(clipped) / math.log1p(maximum)
            output[valid] = np.rint(FAR_VALID_PIXEL_VALUE + (255 - FAR_VALID_PIXEL_VALUE)
                                   * (1.0 - np.clip(fraction, 0.0, 1.0))).astype(np.uint8)
    return Image.fromarray(output).convert("RGB")
