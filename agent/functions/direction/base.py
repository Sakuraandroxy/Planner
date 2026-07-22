"""Direction estimation function interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Tuple


class BaseDirectionEstimator(ABC):
    """Estimate coarse target direction from a detection box."""

    @abstractmethod
    def estimate(self, bbox, camera_id: int, image_size: Tuple[int, int]) -> str:
        ...
