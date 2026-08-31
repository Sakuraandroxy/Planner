"""Focused acceptance tests for arbitrary AirSim camera poses and recording."""

from __future__ import annotations

import math
from types import SimpleNamespace

from PIL import Image

from agent.functions.completion.task_completion import TaskCompletionChecker
from agent.functions.perception.camera_geometry import (
    pixel_depth_to_world,
    project_world_to_pixel,
)
from agent.functions.relocalization.relocalizer import TargetRelocalizer
from sim.airsim_client import AirSimClient
from sim.camera_api_recorder import CameraApiRecorder
from sim.camera_frame_hub import CameraFrameHub
from sim.camera_frames import CameraFrame, CameraIntrinsics, MultiCameraSnapshot
from sim.camera_recording_options import CameraRecordingOptions


def _frame(
    camera_id: str,
    rotation_camera_to_world,
    *,
    position=(1.0, 2.0, -3.0),
    capture_id="capture-1",
) -> CameraFrame:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    return CameraFrame(
        camera_id=camera_id,
        capture_id=capture_id,
        timestamp_ns=123,
        rgb_intrinsics=intrinsics,
        depth_intrinsics=intrinsics,
        camera_position_world=list(position),
        rotation_camera_to_world=rotation_camera_to_world,
    )


def _image_with_frame(frame: CameraFrame):
    image = Image.new("RGB", (640, 480), (80, 80, 80))
    image.camera_frame = frame
    return image


def test_arbitrary_pitch_pixel_depth_round_trip_uses_exact_camera_pose():
    cosine = math.sqrt(0.5)
    # Camera optical +X points 45 degrees down in AirSim NED world.
    rotation = [
        [cosine, 0.0, -cosine],
        [0.0, 1.0, 0.0],
        [cosine, 0.0, cosine],
    ]
    frame = _frame("oblique_camera", rotation)

    world = pixel_depth_to_world(frame, 320.0, 240.0, 10.0)
    projected = project_world_to_pixel(frame, world, require_in_frame=True)

    assert projected is not None
    assert all(
        math.isclose(actual, expected, abs_tol=1e-7)
        for actual, expected in zip(
            world,
            [1.0 + 10.0 * cosine, 2.0, -3.0 + 10.0 * cosine],
        )
    )
    assert math.isclose(projected[0], 320.0, abs_tol=1e-7)
    assert math.isclose(projected[1], 240.0, abs_tol=1e-7)
    assert math.isclose(projected[2], 10.0, abs_tol=1e-7)


def test_camera_roles_select_input_slots_without_implying_orientation():
    roles, specs = AirSimClient._resolve_camera_roles(
        {
            "CAMERAS": {
                "camera_7": {"ROLE": "auxiliary", "FOV": 80},
                "camera_2": {"ROLES": ["primary", "relocalization"], "FOV": 95},
            }
        }
    )

    assert roles["primary"] == "camera_2"
    assert roles["auxiliary"] == "camera_7"
    assert roles["relocalization"] == "camera_2"
    assert specs["camera_2"]["FOV"] == 95


def test_settings_capture_override_preserves_existing_camera_extrinsics(monkeypatch):
    import sim.airsim_settings as settings_module

    existing = {
        "SettingsVersion": 1.2,
        "Vehicles": {
            "DroneTest": {
                "Cameras": {
                    "tilted": {
                        "X": 0.4,
                        "Y": -0.2,
                        "Z": 0.1,
                        "Pitch": -37.5,
                        "Roll": 3.0,
                        "Yaw": 12.0,
                    }
                }
            }
        },
    }
    monkeypatch.setattr(settings_module, "load_local_airsim_settings", lambda: existing)
    monkeypatch.setitem(
        settings_module.cfg,
        "SIM",
        {
            "VEHICLE_NAME": "DroneTest",
            "PRESERVE_CAMERA_POSE_ON_SETTINGS_WRITE": True,
            "CAMERAS": {"tilted": {"ROLE": "primary", "FOV": 88}},
        },
    )

    result = settings_module.build_airsim_settings_with_overrides()
    camera = result["Vehicles"]["DroneTest"]["Cameras"]["tilted"]

    assert camera["Pitch"] == -37.5
    assert camera["Roll"] == 3.0
    assert camera["Yaw"] == 12.0
    assert camera["X"] == 0.4
    assert camera["CaptureSettings"][0]["FOV_Degrees"] == 88.0


def test_relocalization_only_checks_cameras_that_can_project_locked_target():
    facing = _frame(
        "misleading_down_name",
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        position=(0.0, 0.0, 0.0),
    )
    facing_away = _frame(
        "misleading_front_name",
        [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
        position=(0.0, 0.0, 0.0),
    )
    relocalizer = TargetRelocalizer({"RELOCALIZATION": {"ENABLED": True}})

    decisions = relocalizer._predicted_visible_views(
        [_image_with_frame(facing), _image_with_frame(facing_away)],
        [10.0, 0.0, 0.0],
    )

    assert decisions == [True, False]


def test_completion_downward_view_uses_real_optical_axis_not_camera_name():
    actual_down = _frame(
        "front_center",
        [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    )
    actual_horizontal = _frame(
        "down_center",
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    )

    assert TaskCompletionChecker._camera_points_downward(_image_with_frame(actual_down))
    assert not TaskCompletionChecker._camera_points_downward(_image_with_frame(actual_horizontal))


def test_recorder_is_off_by_default_and_creates_directory_on_first_frame(tmp_path):
    output_root = tmp_path / "camera_api"
    options = CameraRecordingOptions(
        enabled=False,
        mode="off",
        output_root=output_root,
        save_metadata=False,
        min_free_disk_gb=0.0,
    )
    hub = CameraFrameHub()
    recorder = CameraApiRecorder(hub, options)

    assert not recorder.status()["enabled"]
    assert recorder.run_directory is None
    assert not output_root.exists()

    recorder.start(mode="frames")
    assert recorder.run_directory is None
    assert not output_root.exists()

    frame = _frame(
        "camera/one",
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        position=(0.0, 0.0, 0.0),
    )
    frame.rgb_png = b"synthetic-png-payload"
    snapshot = MultiCameraSnapshot(
        capture_id=frame.capture_id,
        frames={frame.camera_id: frame},
        vehicle_position_world=[0.0, 0.0, 0.0],
        rotation_body_to_world=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        navigation_yaw_deg=0.0,
    )
    assert hub.publish(snapshot)
    status = recorder.stop(drain=True)

    assert not status["enabled"]
    assert status["written_frames"] == 1
    assert output_root.exists()
    assert (output_root / "camera_one").is_dir()

