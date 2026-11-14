"""Deterministic free-space profile measured from calibrated metric depth.

A trajectory VLM has no metric prior for distances or yaw magnitude in an
image. Free navigation therefore supplies a measured profile instead: for each
bearing sector, the nearest surface that rises above the local ground.

The ground line is estimated per forward-distance bin from the same depth
frame, so the measurement needs no camera-height constant and follows road
slope. The profile is evidence, not a map: a clear sector has no measured
above-ground surface inside the report range, and unseen space stays unknown.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ..domain.depth import INVALID_DEPTH_SENTINEL_M
from ..domain.observation import Observation


@dataclass(frozen=True)
class FreeSpaceProfileSettings:
    sector_width_deg: float = 10.0
    max_range_m: float = 40.0
    distance_bin_m: float = 1.0
    min_points_per_bin: int = 8
    ground_height_percentile: float = 90.0
    min_height_above_ground_m: float = 0.15
    max_height_above_ground_m: float = 2.5
    obstacle_distance_percentile: float = 5.0
    min_obstacle_pixels: int = 3

    def __post_init__(self) -> None:
        for name, value in (
            ("sector width", self.sector_width_deg),
            ("max range", self.max_range_m),
            ("distance bin", self.distance_bin_m),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"free-space {name} must be finite and positive")
        for name, value in (
            ("minimum points per bin", self.min_points_per_bin),
            ("minimum obstacle pixels", self.min_obstacle_pixels),
        ):
            if type(value) is not int or value < 1:
                raise ValueError(f"free-space {name} must be a positive integer")
        for name, value in (
            ("ground height percentile", self.ground_height_percentile),
            ("obstacle distance percentile", self.obstacle_distance_percentile),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not 0.0 <= value <= 100.0
            ):
                raise ValueError(f"free-space {name} must be within 0-100")
        if (
            not math.isfinite(self.min_height_above_ground_m)
            or not math.isfinite(self.max_height_above_ground_m)
            or not 0.0 < self.min_height_above_ground_m
            < self.max_height_above_ground_m
        ):
            raise ValueError(
                "free-space height band must satisfy 0 < min < max"
            )


@dataclass(frozen=True)
class FreeSpaceSector:
    """Nearest above-ground surface inside one bearing sector.

    ``distance_m`` is None when no surface is measured inside the report
    range. ``ground_available`` is False when the sector has no ground
    reference at all, so the measurement is unknown rather than clear.
    """

    bearing_deg: float
    distance_m: float | None
    forward_m: float | None
    right_m: float | None
    ground_available: bool = True


@dataclass(frozen=True)
class FreeSpaceProfile:
    sectors: tuple[FreeSpaceSector, ...]
    max_range_m: float
    min_height_above_ground_m: float

    def _measured(self) -> tuple[FreeSpaceSector, ...]:
        return tuple(
            sector
            for sector in self.sectors
            if sector.ground_available and sector.distance_m is not None
        )

    @property
    def nearest_ahead_m(self) -> float | None:
        ahead = min(
            self.sectors, key=lambda sector: abs(sector.bearing_deg), default=None
        )
        return None if ahead is None else ahead.distance_m

    @property
    def clear_window_deg(self) -> tuple[float, float] | None:
        runs: list[tuple[int, int]] = []
        start = None
        for index, sector in enumerate(self.sectors):
            clear = sector.ground_available and sector.distance_m is None
            if clear and start is None:
                start = index
            elif not clear and start is not None:
                runs.append((start, index - 1))
                start = None
        if start is not None:
            runs.append((start, len(self.sectors) - 1))
        if not runs:
            return None
        best = max(
            runs,
            key=lambda run: (
                run[1] - run[0],
                -abs(
                    self.sectors[(run[0] + run[1]) // 2].bearing_deg
                ),
            ),
        )
        return (self.sectors[best[0]].bearing_deg, self.sectors[best[1]].bearing_deg)


def measure_free_space(
    observation: Observation,
    settings: FreeSpaceProfileSettings | None = None,
) -> FreeSpaceProfile:
    """Measure the nearest above-ground surface per bearing sector."""
    settings = settings or FreeSpaceProfileSettings()
    depth = np.asarray(observation.depth_m, dtype=np.float64)
    intrinsics = observation.depth_intrinsics or observation.intrinsics
    if depth.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("depth image does not match calibrated intrinsics")
    if observation.depth_kind not in ("optical_depth", "ray_range"):
        raise ValueError("unsupported depth kind")

    columns = np.arange(intrinsics.width, dtype=np.float64)[None, :]
    rows = np.arange(intrinsics.height, dtype=np.float64)[:, None]
    right_ratio = (columns - intrinsics.cx) / intrinsics.fx
    down_ratio = (rows - intrinsics.cy) / intrinsics.fy

    valid = (
        np.isfinite(depth)
        & (depth > 0.0)
        & (depth < INVALID_DEPTH_SENTINEL_M)
    )
    if observation.depth_kind == "ray_range":
        norm = np.sqrt(1.0 + right_ratio ** 2 + down_ratio ** 2)
        forward = depth / norm
    else:
        forward = depth
    right = forward * right_ratio
    down = forward * down_ratio
    horizontal = np.hypot(forward, right)
    valid &= (
        np.isfinite(forward)
        & (forward > 0.0)
        & (horizontal <= settings.max_range_m)
    )

    centers = _sector_centers(observation, settings)
    if not valid.any():
        return FreeSpaceProfile(
            tuple(
                FreeSpaceSector(center, None, None, None, False)
                for center in centers
            ),
            settings.max_range_m,
            settings.min_height_above_ground_m,
        )

    forward = forward[valid]
    right = right[valid]
    down = down[valid]
    horizontal = horizontal[valid]
    bearing = np.degrees(np.arctan2(right, forward))
    sector_index = np.rint(bearing / settings.sector_width_deg).astype(np.int64)

    sectors = []
    for center in centers:
        index = int(round(center / settings.sector_width_deg))
        selected = sector_index == index
        sectors.append(
            _measure_sector(
                center,
                forward[selected],
                right[selected],
                down[selected],
                horizontal[selected],
                settings,
            )
        )
    return FreeSpaceProfile(
        tuple(sectors),
        settings.max_range_m,
        settings.min_height_above_ground_m,
    )


def _sector_centers(
    observation: Observation, settings: FreeSpaceProfileSettings
) -> tuple[float, ...]:
    half_fov = observation.intrinsics.horizontal_fov_deg / 2.0
    width = settings.sector_width_deg
    first = math.ceil((-half_fov + 1e-9) / width)
    last = math.floor((half_fov - 1e-9) / width)
    if last < first:
        return (0.0,)
    return tuple(value * width for value in range(first, last + 1))


def _measure_sector(
    center: float,
    forward: np.ndarray,
    right: np.ndarray,
    down: np.ndarray,
    horizontal: np.ndarray,
    settings: FreeSpaceProfileSettings,
) -> FreeSpaceSector:
    if forward.size == 0:
        return FreeSpaceSector(center, None, None, None, False)

    bins = np.floor(forward / settings.distance_bin_m).astype(np.int64)
    ground: dict[int, float] = {}
    for value in np.unique(bins):
        sample = down[bins == value]
        if sample.size >= settings.min_points_per_bin:
            ground[int(value)] = float(
                np.percentile(sample, settings.ground_height_percentile)
            )
    if not ground:
        return FreeSpaceSector(center, None, None, None, False)

    bin_ids = np.arange(
        0, int(math.floor(settings.max_range_m / settings.distance_bin_m)) + 1
    )
    known = np.array(sorted(ground), dtype=np.float64)
    known_height = np.array(
        [ground[int(value)] for value in known], dtype=np.float64
    )
    ground_height = np.interp(bin_ids, known, known_height)

    height_above_ground = ground_height[bins] - down
    obstacle = (
        height_above_ground >= settings.min_height_above_ground_m
    ) & (height_above_ground <= settings.max_height_above_ground_m)
    if int(obstacle.sum()) < settings.min_obstacle_pixels:
        return FreeSpaceSector(center, None, None, None, True)

    distances = horizontal[obstacle]
    order = np.argsort(distances, kind="stable")
    rank = min(
        order.size - 1,
        int(
            math.floor(
                settings.obstacle_distance_percentile / 100.0 * order.size
            )
        ),
    )
    point = order[rank]
    return FreeSpaceSector(
        center,
        float(distances[point]),
        float(forward[obstacle][point]),
        float(right[obstacle][point]),
        True,
    )