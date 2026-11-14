"""Encode numpy RGB images as data URLs for multimodal chat requests."""

from __future__ import annotations

import base64

import cv2
import numpy as np


def resize_longest_side(rgb: np.ndarray, max_side: int) -> np.ndarray:
    """Downscale so the longest side is at most ``max_side`` pixels."""
    array = np.asarray(rgb)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError("expected an HxWx3 RGB array")
    if max_side <= 0:
        raise ValueError("max_side must be positive")
    height, width = array.shape[:2]
    scale = max_side / float(max(height, width))
    if scale >= 1.0:
        return np.ascontiguousarray(array)
    size = (
        max(1, int(round(width * scale))),
        max(1, int(round(height * scale))),
    )
    return cv2.resize(array, size, interpolation=cv2.INTER_AREA)


def encoded_image_payload(
    rgb: np.ndarray, *, fmt: str = "JPEG", quality: int = 90
) -> tuple[str, str]:
    """Return ``(mime_subtype, base64_data)`` for an RGB uint8 array."""
    array = np.ascontiguousarray(np.asarray(rgb, dtype=np.uint8))
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError("expected an HxWx3 RGB array")
    fmt = fmt.upper()
    if fmt == "JPEG":
        payload = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        params = [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
    elif fmt == "PNG":
        payload = array
        params = []
    else:
        raise ValueError("fmt must be JPEG or PNG")
    mime = fmt.lower()
    ok, buffer = cv2.imencode(f".{mime}", payload, params)
    if not ok:
        raise RuntimeError(f"could not encode image as {fmt}")
    return mime, base64.b64encode(buffer.tobytes()).decode("ascii")


def encoded_image_data_url(
    rgb: np.ndarray, *, fmt: str = "JPEG", quality: int = 90
) -> str:
    """Return ``data:image/<fmt>;base64,...`` for an RGB uint8 array."""
    mime, encoded = encoded_image_payload(rgb, fmt=fmt, quality=quality)
    return f"data:image/{mime};base64,{encoded}"


def image_url(image: np.ndarray, image_format: str = "PNG") -> str:
    """Reference-compatible name for numpy RGB image transport encoding."""
    return encoded_image_data_url(image, fmt=image_format)