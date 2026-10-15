from types import SimpleNamespace
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import Mock
import numpy as np
import pytest
from planner.adapters.airsim.connection import AirSimConnection
from planner.adapters.airsim.depth_visualization import render_depth
from planner.adapters.airsim.observation_source import AirSimObservationSource
from planner.adapters.airsim.recording import CameraApiRecorder
from planner.domain.depth import depth_display_description
from planner.errors import ObservationError
from tests.unit.test_camera_recording import ObservationSource


def response(timestamp):
    return SimpleNamespace(time_stamp=timestamp,
        camera_position=SimpleNamespace(x_val=0, y_val=0, z_val=0),
        camera_orientation=SimpleNamespace(x_val=0, y_val=0, z_val=0, w_val=1))


@pytest.mark.parametrize("gap,ok", [(0, True), (50_000_000, True), (50_000_001, False)])
def test_pair_time_gap(gap, ok):
    if ok:
        AirSimObservationSource._validate_pair(response(100), response(100+gap))
    else:
        with pytest.raises(ObservationError, match="timestamps"):
            AirSimObservationSource._validate_pair(response(100), response(100+gap))


def test_versioned_depth_preserves_far_valid_and_invalid_semantics():
    raw = np.array([[0, .2, 2, 200, 300, 65500, np.nan]], dtype=np.float32)
    before = raw.copy()
    image = np.asarray(render_depth(raw, 0, 200, "log1p_inverse_gray_v2"))[0, :, 0]
    assert image[0] == image[5] == image[6] == 0
    assert image[1] > image[2] > image[3] == image[4] == 16
    np.testing.assert_array_equal(raw, before)
    assert "saturate" in depth_display_description("log1p_inverse_gray_v2", 0, 200)
    legacy = np.asarray(render_depth(raw, .5, 200))[0, :, 0]
    assert legacy[1] == legacy[4] == 0


@pytest.mark.parametrize("landed", [True, False, None])
def test_shutdown_only_disarms_confirmed_landed_vehicle(monkeypatch, landed):
    client = Mock()
    client.getMultirotorState.return_value = SimpleNamespace(landed_state=0 if landed else 1)
    if landed is None:
        client.getMultirotorState.side_effect = RuntimeError("state unavailable")
    client.armDisarm.return_value = True
    monkeypatch.setitem(sys.modules, "airsim", SimpleNamespace(
        MultirotorClient=lambda **kwargs: client, LandedState=SimpleNamespace(Landed=0)))
    connection = AirSimConnection("", 41451, 5)
    connection.prepare_vehicle()
    client.reset_mock()
    connection.shutdown()
    connection.shutdown()
    if landed:
        client.armDisarm.assert_called_once_with(False)
        client.enableApiControl.assert_called_once_with(False)
    else:
        client.armDisarm.assert_not_called()
        client.enableApiControl.assert_not_called()
    client.client.close.assert_called_once()


def test_read_only_connection_close_never_controls_vehicle(monkeypatch):
    client = Mock()
    monkeypatch.setitem(sys.modules, "airsim", SimpleNamespace(MultirotorClient=lambda **kwargs: client))
    AirSimConnection("", 41451, 5).shutdown()
    client.hoverAsync.assert_not_called()
    client.armDisarm.assert_not_called()
    client.client.close.assert_called_once()


def test_slow_recording_stop_does_not_allow_second_worker(tmp_path):
    entered, release = threading.Event(), threading.Event()
    original = ObservationSource()
    class SlowSource:
        def capture(self):
            entered.set()
            assert release.wait(2)
            return original.capture()
    recorder = CameraApiRecorder(SlowSource(), tmp_path, 5)
    try:
        with pytest.raises(TimeoutError):
            recorder.start(timeout_s=.05)
        assert entered.is_set() and not recorder.stop(timeout_s=0)
        with pytest.raises(RuntimeError, match="stopping"):
            recorder.start()
    finally:
        release.set()
        assert recorder.close(timeout_s=2)
