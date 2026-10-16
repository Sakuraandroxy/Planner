"""Stage-scoped static target estimate; accepted coordinates are not ground truth."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TargetPositionMemory:
    stage_id: str
    anchor_world_point: tuple[float, float, float]
    world_point: tuple[float, float, float]
    capture_id: str
    captured_monotonic: float
    timestamp_ns: int

    def __post_init__(self):
        if not isinstance(self.stage_id, str) or not self.stage_id.strip() or not isinstance(self.capture_id, str) or not self.capture_id.strip():
            raise ValueError('target memory requires stage and capture identity')
        for point in (self.anchor_world_point, self.world_point):
            if not isinstance(point, tuple) or len(point) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
                raise ValueError('target memory requires finite 3D positions')
        if (isinstance(self.captured_monotonic, bool) or not math.isfinite(self.captured_monotonic)
                or self.captured_monotonic <= 0 or type(self.timestamp_ns) is not int or self.timestamp_ns <= 0):
            raise ValueError('target memory requires positive acquisition times')
