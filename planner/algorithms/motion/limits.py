"""模长限幅：速度、加速度与 jerk 向量的唯一实现（裁决 R22）。"""

from __future__ import annotations

import math


def bounded(vector: tuple[float, ...], maximum: float) -> tuple[float, ...]:
    norm = math.hypot(*vector)
    scale = min(1.0, maximum / norm) if norm else 1.0
    return tuple(value * scale for value in vector)
