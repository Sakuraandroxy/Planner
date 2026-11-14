"""AirSim observation capture must produce the shared ``Observation`` contract.

The adapter used to build ``Observation`` with the old branch's field names
(``depth_meters``, ``vehicle_pose``) and never once ran: the first real capture
raised ``TypeError`` at construction. Nothing caught it because this package had
no tests at all, and ``test_architecture.py`` only checks imports and slot lists.

These tests fake the ``airsim`` module (it is not installed in the web
environment) and drive ``capture()`` for real, so ``Observation.__post_init__``
validates every field the way production will.
"""

import sys
import types

import numpy as np
import pytest

from planner.domain.pose import WorldPose


def _png_bytes(width=8, height=6, color=(10, 20, 30)):
    from PIL import Image
    import io

    image = Image.new("RGB", (width, height), color)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


class _Vec:
    def __init__(self, x, y, z):
        self.x_val, self.y_val, self.z_val = x, y, z


class _Response:
    def __init__(self, width, height, *, rgb=False, depth=False, timestamp_ns):
        self.width, self.height = width, height
        self.time_stamp = timestamp_ns
        self.camera_position = _Vec(1.0, 0.0, -0.5)
        self.camera_orientation = _Quat(0.0, 0.0, 0.0, 1.0)
        self.image_data_uint8 = _png_bytes(width, height) if rgb else b""
        self.image_data_float = (
            np.full((height, width), 4.5, dtype=np.float32).ravel().tolist() if depth else []
        )


class _Quat:
    def __init__(self, x, y, z, w):
        self.x_val, self.y_val, self.z_val, self.w_val = x, y, z, w


class _FakeAirsim(types.ModuleType):
    """Only the pieces ``_capture`` touches."""

    def __init__(self):
        super().__init__("airsim")

        class ImageType:
            Scene = 0
            DepthPerspective = 2

        class ImageRequest:
            def __init__(self, camera, image_type, pixels_as_float, compress):
                self.camera, self.image_type = camera, image_type
                self.pixels_as_float, self.compress = pixels_as_float, compress

        self.ImageType = ImageType
        self.ImageRequest = ImageRequest


class _CameraInfo:
    """What ``simGetCameraInfo`` really answers on the target build: an object
    with ``fov``, ``pose`` and ``proj_mat``. Verified against live UE4."""

    fov = 89.90362548828125


class _Connection:
    def __init__(self, rgb_ts, depth_ts):
        self.calls = []
        # The shared Observation contract requires depth and rgb to cover the same
        # pixels, so a valid rig publishes both at one resolution. AirSim's
        # settings.json on this machine sends Scene 1920x1080 against Depth
        # 256x256, which is rejected -- see test_mismatched_depth_shape_is_rejected.
        self.responses = [
            _Response(8, 6, rgb=True, timestamp_ns=rgb_ts),
            _Response(8, 6, depth=True, timestamp_ns=depth_ts),
        ]

    def call(self, name, args):
        self.calls.append((name, args))
        if name == "simGetCameraInfo":
            return _CameraInfo()
        return self.responses


class _Vehicle:
    holding = None

    def current_pose(self):
        return WorldPose(0.0, 0.0, 0.67, 0.0)


@pytest.fixture(autouse=True)
def fake_airsim(monkeypatch):
    monkeypatch.setitem(sys.modules, "airsim", _FakeAirsim())


def _source(rgb_ts=1_000_000_000, depth_ts=1_000_000_020):
    from planner.simulators.airsim.observation_source import AirSimObservationSource

    return AirSimObservationSource(
        _Connection(rgb_ts, depth_ts), _Vehicle(), "front_center", 0.5, 200.0,
        depth_horizontal_fov_deg=90.0, maximum_pair_time_gap_s=0.05,
    )


def test_capture_returns_a_valid_observation():
    observation = _source().capture()

    assert isinstance(observation.rgb, np.ndarray) or hasattr(observation.rgb, "ndim"), (
        "the shared contract validates rgb.ndim; a PIL image is not an array"
    )
    assert observation.rgb.ndim == 3 and observation.rgb.shape[2] == 3
    assert observation.depth_m.shape == observation.rgb.shape[:2]
    assert observation.frame == 1
    assert observation.timestamp_s == pytest.approx(1.0)
    assert observation.camera_matrix.shape == (4, 4)
    assert observation.camera_id == "front_center"
    assert observation.depth_kind in ("optical_depth", "ray_range")


def test_frames_are_numbered_in_order():
    source = _source()
    assert [source.capture().frame for _ in range(3)] == [1, 2, 3]


def test_depth_is_reported_in_metres_from_the_response():
    observation = _source().capture()
    assert float(np.nanmax(observation.depth_m)) == pytest.approx(4.5)


def test_a_depth_response_that_lies_about_its_size_is_rejected():
    """``reshape`` is where a short float array shows up, wrapped as ObservationError.

    Depth no longer has to match the rgb grid -- that requirement was what forced
    metric depth to 1280x720 and a 25 s capture. What still has to hold is that the
    buffer is as long as the response claims.
    """
    from planner.errors import ObservationError

    source = _source()
    lying = _Response(4, 3, depth=True, timestamp_ns=1_000_000_020)
    lying.image_data_float = lying.image_data_float[:6]      # claims 12, sends 6
    source.connection.responses[1] = lying
    with pytest.raises(ObservationError, match="cannot reshape array of size 6"):
        source.capture()


def test_a_smaller_depth_grid_is_accepted_and_calibrated():
    """The point of the shared contract change: rgb large, depth small, one FOV."""
    source = _source()
    source.depth_horizontal_fov_deg = None      # depth follows the camera's own FOV
    source.connection.responses[1] = _Response(4, 3, depth=True, timestamp_ns=1_000_000_020)
    observation = source.capture()

    assert observation.depth_m.shape == (3, 4)
    assert observation.rgb.shape[:2] == (6, 8), "rgb keeps its own grid"
    depth = observation.depth_intrinsics
    assert (depth.width, depth.height) == (4, 3)
    assert depth.fx == pytest.approx(observation.intrinsics.fx / 2.0), (
        "same FOV at half the width means half the focal length in pixels"
    )
    assert observation.depth_image.shape[:2] == observation.depth_m.shape


def test_incomplete_pair_is_rejected():
    source = _source()
    source.connection.responses[0].image_data_uint8 = b""
    with pytest.raises(Exception, match="incomplete RGB/depth pair"):
        source.capture()


def test_timestamp_gap_beyond_the_pair_window_is_rejected():
    # 0.05 s window, 0.5 s apart: the two images are not the same moment.
    source = _source(rgb_ts=1_000_000_000, depth_ts=1_500_000_000)
    source.verify_image_pair = True
    with pytest.raises(Exception, match="pair gap|not registered|missing"):
        source.capture()
