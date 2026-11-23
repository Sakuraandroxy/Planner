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

import math
from dataclasses import dataclass

import numpy as np

from planner.domain import INVALID_DEPTH_SENTINEL_M
from planner.domain import Observation


@dataclass(frozen=True)
class FreeSpaceProfileSettings:
    """自由空间剖面的九个门限。

    每项都注了出处。标「未标定」的都是经验值：代码库里没有对应的实测记录，
    改之前先按注释里的影响因素跑一次剖面回归（`tests/test_free_space.py` 只校参数）。
    """

    #: 扇面宽度。出处：经验值，未标定。影响因素：相机水平 FOV、深度图宽度（当前
    #: 256 px）、每扇面落到 `min_points_per_bin` 所需的点数。
    sector_width_deg: float = 10.0
    #: 剖面报告距离上限。出处：经验值，未标定；受深度有效量程与规划前瞻距离约束，
    #: 超出后深度只剩天空饱和值（`INVALID_DEPTH_SENTINEL_M`）。影响因素：深度量程、
    #: 前瞻距离、路面坡度造成的远处地面点稀疏。
    max_range_m: float = 40.0
    #: 地面线分箱粒度。出处：经验值，未标定。影响因素：地面坡度变化的空间尺度、
    #: 每个箱子里的点数与 `min_points_per_bin` 的关系。
    distance_bin_m: float = 1.0
    #: 一个箱子算有效地面采样所需点数。出处：经验值，未标定。影响因素：深度分辨率、
    #: 量程（越远越稀）、相机俯仰与地面遮挡。
    min_points_per_bin: int = 8
    #: 地面高度取的分位数。出处：经验值，未标定。影响因素：地面点的高程噪声、
    #: 坡面倾角、地面材质造成的丢点。
    ground_height_percentile: float = 90.0
    #: 障碍高度带下沿。出处：经验值，未标定；受载具能压过或跨过的高度约束。
    #: 影响因素：车高、离地间隙、路面噪点起伏。
    min_height_above_ground_m: float = 0.15
    #: 障碍高度带上沿。出处：经验值，未标定；受载具能通过的高度约束。
    #: 影响因素：车高与相机高度、可通行净空。
    max_height_above_ground_m: float = 2.5
    #: 障碍距离取的分位数。出处：经验值，未标定。影响因素：障碍像素数、深度噪声
    #: 造成的离群点。
    obstacle_distance_percentile: float = 5.0
    #: 判定为障碍所需像素数。出处：经验值，未标定。影响因素：深度分辨率、障碍距离
    #: （同一物体越远像素越少）、部分遮挡。
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
        index = round(center / settings.sector_width_deg)
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
        0, math.floor(settings.max_range_m / settings.distance_bin_m) + 1
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
        math.floor(
                settings.obstacle_distance_percentile / 100.0 * order.size
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