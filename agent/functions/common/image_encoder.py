"""Shared image encoding cache for detector and planner calls."""

from __future__ import annotations

import base64
from io import BytesIO
from typing import Optional

_front_jpeg_b64: Optional[str] = None
_down_jpeg_b64: Optional[str] = None


class ImageEncoder:
    """Encode PIL images to JPEG base64 and store the latest front/down cache."""

    @staticmethod
    def encode_front(frame) -> Optional[str]:
        global _front_jpeg_b64
        _front_jpeg_b64 = _encode_image(frame)
        return _front_jpeg_b64

    @staticmethod
    def encode_down(down_frame) -> Optional[str]:
        global _down_jpeg_b64
        _down_jpeg_b64 = _encode_image(down_frame)
        return _down_jpeg_b64

    @staticmethod
    def clear():
        global _front_jpeg_b64, _down_jpeg_b64
        _front_jpeg_b64 = None
        _down_jpeg_b64 = None


def _encode_image(image) -> Optional[str]:
    if image is None:
        return None
    if image.mode == "RGBA":
        image = image.convert("RGB")
    elif image.mode != "RGB":
        image = image.convert("RGB")
    buf = BytesIO()
    image.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode("utf-8")


def get_cached_front_b64() -> Optional[str]:
    return _front_jpeg_b64


def get_cached_down_b64() -> Optional[str]:
    return _down_jpeg_b64


def has_cached_front() -> bool:
    return _front_jpeg_b64 is not None


def has_cached_down() -> bool:
    return _down_jpeg_b64 is not None
