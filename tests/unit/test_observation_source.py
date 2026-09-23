import io
import sys
from types import SimpleNamespace
from unittest.mock import Mock

from PIL import Image

from planner.adapters.airsim.observation_source import AirSimObservationSource
from planner.domain.pose import WorldPose


def _png_bytes() -> bytes:
    output = io.BytesIO()
    Image.new("RGB", (2, 2), "white").save(output, format="PNG")
    return output.getvalue()


def _vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x_val=x, y_val=y, z_val=z)


def _quaternion():
    return SimpleNamespace(x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0)


def test_capture_caches_fov_and_uses_image_response_camera_pose(monkeypatch):
    monkeypatch.setitem(sys.modules, "airsim", SimpleNamespace(
        ImageRequest=lambda *args: args,
        ImageType=SimpleNamespace(Scene=0, DepthPerspective=1),
    ))
    rgb_response = SimpleNamespace(
        image_data_uint8=_png_bytes(),
        camera_position=_vector(1.0, 2.0, 3.0),
        camera_orientation=_quaternion(),
        time_stamp=123,
    )
    depth_response = SimpleNamespace(
        image_data_float=[1.0, 2.0, 3.0, 4.0], height=2, width=2,
    )
    connection = Mock()

    def call(name, *args):
        if name == "simGetImages":
            return [rgb_response, depth_response]
        if name == "simGetCameraInfo":
            return SimpleNamespace(fov=90.0)
        raise AssertionError(f"unexpected AirSim call: {name}")

    connection.call.side_effect = call
    vehicle = Mock()
    vehicle.current_pose.return_value = WorldPose(10.0, 20.0, -5.0, 30.0)
    source = AirSimObservationSource(connection, vehicle, "front_center", 0.5, 200.0)

    first = source.capture()
    second = source.capture()

    assert first.camera_position_world == (1.0, 2.0, 3.0)
    assert second.intrinsics.horizontal_fov_deg == 90.0
    assert [item.args[0] for item in connection.call.call_args_list].count("simGetCameraInfo") == 1
    assert vehicle.current_pose.call_count == 2
