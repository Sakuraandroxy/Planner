"""运动律入口：模长限幅、越点走廊与越点判据（裁决 R6、R19a）。"""

from .corridor import corridor_default, passed_lateral_corridor
from .limits import bounded
from .reached import reached

__all__ = ["bounded", "corridor_default", "passed_lateral_corridor", "reached"]
