"""MJPEG framing on top of the latest-frame hub.

One async generator per viewer. The generator only polls the hub briefly, so
disconnects and server shutdown cancel it promptly; JPEG encoding runs on the
threadpool, keeping simulator callbacks free of network work.

Each frame is followed immediately by the next part delimiter and headers.
Chromium only paints a multipart image once it has seen the boundary that
terminates the part, so without the trailing delimiter an idle stream (a
single warm-up frame) would never be shown until the connection closes.
"""

from __future__ import annotations

import asyncio
from functools import partial

import cv2
import numpy as np
from starlette.concurrency import run_in_threadpool

from ...services.frame_hub import FrameHub

BOUNDARY = b"frame"
_DELIMITER = b"--" + BOUNDARY + b"\r\n"
_PART_HEADERS = b"Content-Type: image/jpeg\r\n\r\n"
_CLOSING = b"\r\n--" + BOUNDARY + b"--\r\n"


def encode_jpeg(bgra: np.ndarray, *, quality: int = 80) -> bytes:
    bgr = cv2.cvtColor(np.asarray(bgra), cv2.COLOR_BGRA2BGR)
    ok, encoded = cv2.imencode(
        ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return encoded.tobytes()


async def mjpeg_chunks(
    frames: FrameHub,
    *,
    quality: int = 80,
    poll_s: float = 0.1,
):
    """Yield ``multipart/x-mixed-replace`` parts until the viewer goes away."""
    sequence = 0
    started = False
    while not frames.closed:
        item = frames.wait_next(sequence, timeout_s=0.0)
        if item is None:
            # Bounded sleeps keep the generator cancellable on disconnect.
            await asyncio.sleep(poll_s)
            continue
        sequence, bgra = item
        payload = await run_in_threadpool(
            partial(encode_jpeg, bgra, quality=quality)
        )
        if started:
            # The previous part already has headers; append its JPEG and open
            # the next part so this frame is complete for the parser.
            yield payload + b"\r\n" + _DELIMITER + _PART_HEADERS
        else:
            started = True
            yield (
                _DELIMITER + _PART_HEADERS
                + payload + b"\r\n"
                + _DELIMITER + _PART_HEADERS
            )
    if started:
        yield _CLOSING