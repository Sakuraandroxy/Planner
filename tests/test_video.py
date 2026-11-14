import asyncio
import time

import cv2
import numpy as np
import pytest

from planner.adapters.http.video import encode_jpeg, mjpeg_chunks
from planner.services.frame_hub import FrameHub


def _bgra(width=8, height=6) -> np.ndarray:
    frame = np.zeros((height, width, 4), dtype=np.uint8)
    frame[..., 0] = 40
    frame[..., 1] = 120
    frame[..., 2] = 200
    frame[..., 3] = 255
    return frame


_PART_HEAD = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
_NEXT_PART = b"\r\n--frame\r\nContent-Type: image/jpeg\r\n\r\n"


def _next_chunk(hub, **kwargs) -> bytes:
    async def run():
        chunks = mjpeg_chunks(hub, **kwargs)
        try:
            return await anext(chunks)
        finally:
            await chunks.aclose()

    return asyncio.run(run())


def _decode(payload: bytes) -> np.ndarray:
    return cv2.imdecode(
        np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
    )


def test_encode_jpeg_returns_a_decodable_image():
    payload = encode_jpeg(_bgra())
    assert payload.startswith(b"\xff\xd8")
    image = cv2.imdecode(
        np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert image.shape == (6, 8, 3)


def test_mjpeg_chunk_wraps_the_first_frame():
    hub = FrameHub(max_fps=1000)
    hub.publish(_bgra())
    chunk = _next_chunk(hub, poll_s=0.01)
    assert chunk.startswith(_PART_HEAD)
    # The part only completes for the browser when the next delimiter arrives,
    # so the first chunk already opens the following part.
    assert chunk.endswith(_NEXT_PART)
    image = _decode(chunk[len(_PART_HEAD) : -len(_NEXT_PART)])
    assert image.shape == (6, 8, 3)


def test_mjpeg_later_chunks_close_the_previous_part():
    async def run():
        hub = FrameHub(max_fps=1000)
        hub.publish(_bgra())
        chunks = mjpeg_chunks(hub, poll_s=0.01)
        await anext(chunks)
        time.sleep(0.002)
        hub.publish(_bgra())
        try:
            return await anext(chunks)
        finally:
            await chunks.aclose()

    chunk = asyncio.run(run())
    assert not chunk.startswith(_PART_HEAD)
    assert chunk.endswith(b"\r\n" + _PART_HEAD)
    image = _decode(chunk[: -len(_NEXT_PART)])
    assert image.shape == (6, 8, 3)


def test_mjpeg_starts_from_the_newest_frame():
    hub = FrameHub(max_fps=1000)
    first = _bgra()
    first[..., 2] = 10
    second = _bgra()
    second[..., 2] = 250
    hub.publish(first)
    time.sleep(0.002)
    hub.publish(second)
    chunk = _next_chunk(hub, poll_s=0.01)
    image = _decode(chunk[len(_PART_HEAD) : -len(_NEXT_PART)])
    assert int(image[0, 0, 2]) > 200


def test_mjpeg_generator_is_cancellable_while_idle():
    async def run():
        hub = FrameHub(max_fps=10)
        chunks = mjpeg_chunks(hub, poll_s=0.01)
        pending = asyncio.ensure_future(anext(chunks))
        await asyncio.sleep(0.05)
        pending.cancel()
        with pytest.raises(asyncio.CancelledError):
            await pending
        await chunks.aclose()

    asyncio.run(run())


def test_mjpeg_stops_when_the_hub_closes():
    async def run():
        hub = FrameHub(max_fps=10)
        hub.publish(_bgra())
        chunks = mjpeg_chunks(hub, poll_s=0.01)
        await anext(chunks)
        hub.close()
        assert await anext(chunks) == b"\r\n--frame--\r\n"
        with pytest.raises(StopAsyncIteration):
            await anext(chunks)
        await chunks.aclose()

    asyncio.run(run())


def test_mjpeg_closed_hub_yields_nothing():
    async def run():
        hub = FrameHub(max_fps=10)
        hub.close()
        chunks = mjpeg_chunks(hub, poll_s=0.01)
        with pytest.raises(StopAsyncIteration):
            await anext(chunks)
        await chunks.aclose()

    asyncio.run(run())