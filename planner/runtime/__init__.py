"""High-level services (motion planning, later the VLN loop)."""

from .segment_motion import (
    SegmentSetpoint,
    duration_for,
    plan_segment,
    quintic,
)

__all__ = [
    "SegmentSetpoint",
    "duration_for",
    "plan_segment",
    "quintic",
]