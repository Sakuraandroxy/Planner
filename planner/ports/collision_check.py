"""Optional control-thread collision source; no read means no collision evidence."""
from typing import Protocol


class CollisionCheck(Protocol):
    def __call__(self) -> bool:
        """Read a collision event; propagate source failures to the workflow."""
        ...
