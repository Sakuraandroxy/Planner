import math

import numpy as np
import pytest

from planner.adapters.projectairsim.camera import (
    assert_camera_matches_scene,
    capture_settings,
    mount_from_sensor,
    sensor_configuration,
)
from planner.adapters.projectairsim.coordinates import (
    body_pose_from_camera,
    camera_matrix,
    camera_position_from_message,
    pose_from_kinematics,
    quaternion_to_rotation,
    rotation_from_camera_message,
    rotation_yaw_deg,
    rpy_rotation_deg,
)
from planner.domain.pose import WorldPose


def _scene(camera_origin=("1.8 0.0 0.2", "0 -3 0"), width=1280, height=720, fov=90):
    return {
        "actors": [
            {
                "name": "Rover1",
                "robot-config": {
                    "sensors": [
                        {
                            "id": "FrontCamera",
                            "type": "camera",
                            "origin": {"xyz": camera_origin[0], "rpy-deg": camera_origin[1]},
                            "capture-settings": [
                                {
                                    "image-type": 0,
                                    "width": width,
                                    "height": height,
                                    "fov-degrees": fov,
                                },
                                {
                                    "image-type": 2,
                                    "width": width,
                                    "height": height,
                                    "fov-degrees": fov,
                                },
                            ],
                        }
                    ]
                },
            }
        ]
    }


def test_identity_quaternion_is_the_identity_rotation():
    assert np.allclose(quaternion_to_rotation(0.0, 0.0, 0.0, 1.0), np.eye(3))


def test_yaw_quaternion_turns_forward_toward_positive_y():
    # +90 deg about NED +z maps forward (1, 0, 0) to right/east (0, 1, 0).
    rotation = quaternion_to_rotation(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert np.allclose(rotation @ (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), atol=1e-12)
    assert rotation_yaw_deg(rotation) == pytest.approx(90.0)


def test_mount_pitch_matches_the_published_rover_camera_quaternion():
    # Live message values: quaternion for the configured rpy "0 -3 0".
    measured = quaternion_to_rotation(
        0.0, -0.026176949962973595, 0.0, 0.9996573328971863
    )
    configured = rpy_rotation_deg((0.0, -3.0, 0.0))
    assert np.allclose(measured, configured, atol=1e-9)
    # NED z is down: a downward-pitched camera has a positive forward z.
    forward = configured @ np.array((1.0, 0.0, 0.0))
    assert forward[2] == pytest.approx(math.sin(math.radians(3.0)), abs=1e-12)


def test_rpy_rotation_applies_roll_pitch_then_yaw():
    roll, pitch, yaw = (math.radians(value) for value in (10.0, 20.0, 30.0))

    def rx(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array(((1, 0, 0), (0, c, -s), (0, s, c)))

    def ry(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))

    def rz(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))

    expected = rz(yaw) @ ry(pitch) @ rx(roll)
    assert np.allclose(rpy_rotation_deg((10.0, 20.0, 30.0)), expected, atol=1e-12)


@pytest.mark.parametrize(
    "quaternion",
    [
        (0.0, 0.0, 0.0, 2.0),
        (math.nan, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, float("inf")),
    ],
)
def test_invalid_quaternions_are_rejected(quaternion):
    with pytest.raises(ValueError):
        quaternion_to_rotation(*quaternion)


def test_body_pose_round_trips_through_the_camera_mount():
    body = WorldPose(50.0, 3.0, 0.94, 25.0)
    mount_position = (1.8, 0.0, 0.2)
    mount_rotation = rpy_rotation_deg((0.0, -3.0, 0.0))
    body_rotation = rpy_rotation_deg((0.0, 0.0, body.yaw_deg))
    camera_rotation = body_rotation @ mount_rotation
    camera_position = np.array((body.x, body.y, body.z)) + body_rotation @ np.array(
        mount_position
    )
    recovered = body_pose_from_camera(
        tuple(camera_position), camera_rotation, mount_position, mount_rotation
    )
    assert recovered.as_tuple() == pytest.approx(body.as_tuple(), abs=1e-9)


def test_camera_matrix_carries_rotation_and_ned_translation():
    rotation = rpy_rotation_deg((0.0, -3.0, 15.0))
    matrix = camera_matrix((51.8, 0.0, 1.14), rotation)
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[:3, :3], rotation)
    assert np.allclose(matrix[:3, 3], (51.8, 0.0, 1.14))
    assert np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0))


def test_camera_messages_reject_missing_fields():
    with pytest.raises(ValueError):
        rotation_from_camera_message({"rot_w": 1.0})
    with pytest.raises(ValueError):
        camera_position_from_message({"pos_x": 1.0})


def test_pose_from_kinematics_reads_the_ned_pose():
    kinematics = {
        "time_stamp": 2_013_000_000,
        "pose": {
            "position": {"x": 50.0, "y": -2.5, "z": 0.94},
            "orientation": {
                "w": math.cos(math.radians(15.0) / 2),
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(math.radians(15.0) / 2),
            },
        },
    }
    pose = pose_from_kinematics(kinematics)
    assert pose.as_tuple() == pytest.approx((50.0, -2.5, 0.94, 15.0))


def test_pose_from_kinematics_rejects_malformed_responses():
    with pytest.raises(ValueError):
        pose_from_kinematics({"pose": {"position": {}}})


def test_scene_camera_configuration_is_matched_by_the_adapter_settings():
    scene = _scene()
    sensor = sensor_configuration(scene, "Rover1", "FrontCamera")
    offset, rpy = mount_from_sensor(sensor)
    assert offset == (1.8, 0.0, 0.2)
    assert rpy == (0.0, -3.0, 0.0)
    assert sorted(capture_settings(sensor)) == [0, 2]
    assert_camera_matches_scene(
        sensor,
        width=1280,
        height=720,
        hfov_deg=90.0,
        offset_m=offset,
        mount_rpy_deg=rpy,
    )


@pytest.mark.parametrize(
    "camera, kwargs",
    [
        (_scene(camera_origin=("2.0 0.0 0.2", "0 -3 0")), {}),
        (_scene(camera_origin=("1.8 0.0 0.2", "0 0 0")), {}),
        (_scene(width=640), {}),
        (_scene(fov=120), {}),
        (_scene(), {"offset_m": (1.8, 0.0, -0.2)}),
    ],
)
def test_config_scene_camera_mismatches_are_rejected(camera, kwargs):
    sensor = sensor_configuration(camera, "Rover1", "FrontCamera")
    with pytest.raises(ValueError):
        assert_camera_matches_scene(
            sensor,
            width=1280,
            height=720,
            hfov_deg=90.0,
            offset_m=kwargs.get("offset_m", (1.8, 0.0, 0.2)),
            mount_rpy_deg=kwargs.get("mount_rpy_deg", (0.0, -3.0, 0.0)),
        )


def test_scene_lookup_reports_missing_robot_and_camera():
    with pytest.raises(ValueError):
        sensor_configuration(_scene(), "Rover2", "FrontCamera")
    with pytest.raises(ValueError):
        sensor_configuration(_scene(), "Rover1", "RearCamera")


def test_missing_depth_capture_setting_is_rejected():
    scene = _scene()
    sensor = scene["actors"][0]["robot-config"]["sensors"][0]
    sensor["capture-settings"] = sensor["capture-settings"][:1]
    with pytest.raises(ValueError):
        assert_camera_matches_scene(
            sensor,
            width=1280,
            height=720,
            hfov_deg=90.0,
            offset_m=(1.8, 0.0, 0.2),
            mount_rpy_deg=(0.0, -3.0, 0.0),
        )