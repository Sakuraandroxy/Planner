"""Versioned depth display contract. Raw metric depth remains unchanged."""
import math

LEGACY_DEPTH_MAPPING = "log_inverse_gray_v1"
SHARED_DEPTH_MAPPING = "log1p_inverse_gray_v2"
INVALID_DEPTH_SENTINEL_M = 65500.0
FAR_VALID_PIXEL_VALUE = 16


def validate_depth_protocol(mapping: str, minimum: float, maximum: float) -> None:
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in (minimum, maximum)):
        raise ValueError("depth bounds must be finite numbers")
    if mapping == LEGACY_DEPTH_MAPPING:
        if not 0 < minimum < maximum:
            raise ValueError("legacy depth range must satisfy 0 < min_m < max_m")
    elif mapping == SHARED_DEPTH_MAPPING:
        if minimum != 0 or maximum != 200:
            raise ValueError("log1p_inverse_gray_v2 requires depth range 0-200m")
    else:
        raise ValueError("unsupported depth mapping")


def depth_display_description(mapping, minimum, maximum):
    validate_depth_protocol(mapping, minimum, maximum)
    if mapping == SHARED_DEPTH_MAPPING:
        return (f"Depth display range: {minimum:g}-{maximum:g} m; log1p inverse grayscale. "
                "Brighter pixels mean nearer surfaces; darker gray means farther surfaces. "
                "Valid depths beyond this range saturate to dark gray. Black means invalid depth.")
    return (f"Depth display range: {minimum:g}-{maximum:g} m; legacy log inverse grayscale. "
            "Brighter pixels mean nearer surfaces; darker gray means farther surfaces. "
            "Black means invalid depth or depth outside the display range.")
