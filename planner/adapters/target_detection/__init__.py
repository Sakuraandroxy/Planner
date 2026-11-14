"""Target detection adapters."""

from .client import OpenAICompatibleTargetDetector, parse_target_detection
from .manual import ManualTargetDetector

__all__ = [
    "ManualTargetDetector",
    "OpenAICompatibleTargetDetector",
    "parse_target_detection",
]