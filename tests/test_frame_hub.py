import threading
import time

import numpy as np
import pytest

from planner.services.frame_hub import FrameHub


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


def _frame(value: int) -> np.ndarray:
    return np.full((2, 3, 4), value, dtype=np.uint8)


def test_publish_and_read_latest():
    hub = FrameHub(max_fps=10)
    assert hub.wait_next(0, timeout_s=0.0) is None
    hub.publish(_frame(7), 1.5)
    sequence, frame = hub.wait_next(0)
    assert sequence == 1
    assert frame[0, 0, 0] == 7
    assert hub.sequence == 1
    assert hub.wait_next(1, timeout_s=0.0) is None


def test_publish_is_throttled_by_max_fps():
    clock = FakeClock()
    hub = FrameHub(max_fps=10, clock=clock)
    hub.publish(_frame(1))
    clock.value = 0.05
    hub.publish(_frame(2))
    assert hub.sequence == 1
    clock.value = 0.11
    hub.publish(_frame(3))
    assert hub.sequence == 2
    _, frame = hub.wait_next(1)
    assert frame[0, 0, 0] == 3


def test_waiter_is_woken_by_a_later_publish():
    hub = FrameHub(max_fps=1000)
    thread = threading.Thread(
        target=lambda: (time.sleep(0.05), hub.publish(_frame(9)))
    )
    thread.start()
    started = time.monotonic()
    item = hub.wait_next(0, timeout_s=1.0)
    elapsed = time.monotonic() - started
    thread.join()
    assert item is not None
    assert item[0] == 1
    assert elapsed < 0.5


def test_invalid_max_fps_is_rejected():
    with pytest.raises(ValueError):
        FrameHub(max_fps=0)


def test_close_stops_publishing():
    hub = FrameHub(max_fps=10)
    hub.publish(_frame(1))
    hub.close()
    assert hub.closed
    hub.publish(_frame(2))
    assert hub.sequence == 1