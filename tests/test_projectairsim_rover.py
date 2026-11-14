import math

import pytest

from planner.adapters.projectairsim.rover import (
    SETTLE_DURATION_S,
    ProjectAirSimRover,
)
from planner.config import CameraSettings, DepthSettings
from planner.domain.pose import WorldPose
from planner.errors import ExecutionError
from planner.services.segment_motion import MotionLimits as ConfiguredMotionLimits

WIDTH, HEIGHT = 4, 3
ROBOT_TOPIC = "/Sim/SceneTest/robots/Rover1"
COLLISION_TOPIC = f"{ROBOT_TOPIC}/collision_info"
KINEMATICS_TOPIC = f"{ROBOT_TOPIC}/GetGroundTruthKinematics"
CONTROLS_TOPIC = f"{ROBOT_TOPIC}/SetRoverControls"
SET_POSE_TOPIC = f"{ROBOT_TOPIC}/SetPose"
WORLD_TOPIC = "/Sim/SceneTest"
SET_OBJECT_POSE_TOPIC = f"{WORLD_TOPIC}/SetObjectPose"

CAMERA = CameraSettings(
    width=WIDTH,
    height=HEIGHT,
    hfov_deg=90.0,
    offset_m=(1.8, 0.0, 0.2),
    mount_rpy_deg=(0.0, -3.0, 0.0),
)


def scene_configuration():
    return {
        "actors": [
            {
                "name": "Rover1",
                "robot-config": {
                    "sensors": [
                        {
                            "id": "FrontCamera",
                            "type": "camera",
                            "origin": {"xyz": "1.8 0.0 0.2", "rpy-deg": "0 -3 0"},
                            "capture-settings": [
                                {
                                    "image-type": 0,
                                    "width": WIDTH,
                                    "height": HEIGHT,
                                    "fov-degrees": 90,
                                },
                                {
                                    "image-type": 2,
                                    "width": WIDTH,
                                    "height": HEIGHT,
                                    "fov-degrees": 90,
                                },
                            ],
                        }
                    ]
                },
            }
        ]
    }


class FakeClient:
    def __init__(self):
        self.callbacks = {}
        self.unsubscribed = []

    def subscribe(self, topic, callback):
        self.callbacks[topic] = callback

    def unsubscribe(self, topics):
        self.unsubscribed.append(topics)


class FakeConnection:
    def __init__(self):
        self.client = FakeClient()
        self.pose = (50.0, 0.0, 0.94, 0.0)
        self.time_ns = 1_000_000_000
        self.requests = []
        self.advances = []

    def scene_configuration(self):
        return scene_configuration()

    def sim_time_ns(self):
        return self.time_ns

    def request(self, topic, payload=None):
        self.requests.append((topic, payload))
        if topic == KINEMATICS_TOPIC:
            x, y, z, yaw = self.pose
            half = math.radians(yaw) / 2
            return {
                "time_stamp": self.time_ns,
                "pose": {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": {
                        "w": math.cos(half),
                        "x": 0.0,
                        "y": 0.0,
                        "z": math.sin(half),
                    },
                },
            }
        return {}

    def advance_for_sim_time(self, delta_s):
        self.advances.append(delta_s)
        self.time_ns += int(round(delta_s * 1e9))

    def control_requests(self):
        return [payload for topic, payload in self.requests if topic == CONTROLS_TOPIC]


class FakeRobot:
    name = "Rover1"
    parent_topic = ROBOT_TOPIC
    world_parent_topic = WORLD_TOPIC
    robot_info = {"collision_info": COLLISION_TOPIC}

    def __init__(self):
        self.calls = []

    def enable_api_control(self):
        self.calls.append("enable_api_control")
        return True

    def arm(self):
        self.calls.append("arm")
        return True


def make_rover(*, motion=None):
    connection = FakeConnection()
    robot = FakeRobot()
    rover = ProjectAirSimRover(
        connection,
        robot,
        camera=CAMERA,
        depth=DepthSettings(),
        camera_id="FrontCamera",
        motion=motion,
    )
    return rover, connection, robot


def test_start_enables_api_control_arms_and_holds_the_brake():
    rover, connection, robot = make_rover()
    rover.start()
    assert robot.calls == ["enable_api_control", "arm"]
    assert connection.control_requests()[-1] == {
        "engine": 0.0,
        "steering_angle": 0.0,
        "brake": 1.0,
    }
    rover.close()


def test_stream_velocity_maps_the_reference_and_advances_one_step():
    rover, connection, _ = make_rover()
    rover.start()
    connection.requests.clear()
    rover.stream_velocity((2.0, 0.0, 0.0), 0.0, 0.05)
    assert connection.requests[-1][0] == CONTROLS_TOPIC
    controls = connection.requests[-1][1]
    assert controls["engine"] == pytest.approx(2.0 / 23.3 + 0.004 * 2 + 0.1 * (2 * 0.05))
    assert controls["steering_angle"] == pytest.approx(0.0)
    assert controls["brake"] == pytest.approx(0.0)
    assert connection.advances == [0.05]
    rover.close()


def test_stream_velocity_requires_api_control():
    rover, _, _ = make_rover()
    with pytest.raises(ExecutionError, match="start"):
        rover.stream_velocity((1.0, 0.0, 0.0), 0.0, 0.05)
    rover.close()


def test_stream_velocity_rejects_bad_requests():
    rover, _, _ = make_rover()
    rover.start()
    with pytest.raises(ExecutionError):
        rover.stream_velocity((float("nan"), 0.0, 0.0), 0.0, 0.05)
    with pytest.raises(ExecutionError):
        rover.stream_velocity((1.0, 0.0, 0.0), 0.0, 0.0)
    with pytest.raises(ExecutionError):
        rover.stream_velocity((1.0, 0.0), 0.0, 0.05)
    rover.close()


def test_hover_brakes_and_advances_one_control_period():
    rover, connection, _ = make_rover()
    rover.start()
    connection.requests.clear()
    connection.advances.clear()
    rover.hover()
    assert connection.control_requests()[-1] == {
        "engine": 0.0,
        "steering_angle": 0.0,
        "brake": 1.0,
    }
    assert connection.advances == [pytest.approx(0.05)]
    rover.close()


def test_settle_brakes_and_advances_through_the_spawn_drop():
    rover, connection, _ = make_rover()
    rover.start()
    connection.requests.clear()
    connection.advances.clear()
    rover.settle()
    assert connection.control_requests()[-1] == {
        "engine": 0.0,
        "steering_angle": 0.0,
        "brake": 1.0,
    }
    assert connection.advances == [pytest.approx(SETTLE_DURATION_S)]
    rover.close()


def test_settle_requires_start():
    rover, _, _ = make_rover()
    with pytest.raises(ExecutionError, match="start"):
        rover.settle()
    rover.close()


def test_execute_segment_requires_start():
    rover, _, _ = make_rover()
    with pytest.raises(ExecutionError, match="start"):
        rover.execute_segment(None)
    rover.close()


def test_collision_state_latches_one_non_ground_contact():
    rover, connection, _ = make_rover()
    callback = connection.client.callbacks[COLLISION_TOPIC]
    callback(None, {"object_name": "Landscape_1"})
    assert rover.collision_state() is False
    callback(
        None,
        {
            "object_name": "Building_3",
            "time_stamp": 500,
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "normal": {"x": 0.0, "y": 0.0, "z": -1.0},
            "penetration_depth": 0.2,
        },
    )
    assert rover.collision_state() is True
    assert rover.collision_state() is False
    assert rover.last_collision.object_name == "Building_3"
    assert rover.last_collision.timestamp_ns == 500
    rover.close()


def test_return_home_teleports_to_the_spawn_pose():
    rover, connection, _ = make_rover()
    connection.pose = (60.0, 5.0, 0.94, 90.0)
    home = rover.return_home()
    assert home.x == pytest.approx(50.0)
    assert home.y == pytest.approx(0.0)
    topic, payload = connection.requests[-2]
    assert topic == SET_POSE_TOPIC
    assert payload["pose"]["frame_id"] == "DEFAULT_FRAME"
    assert payload["pose"]["translation"] == {
        "x": pytest.approx(50.0),
        "y": pytest.approx(0.0),
        "z": pytest.approx(0.94),
    }
    assert payload["pose"]["rotation"]["w"] == pytest.approx(1.0)
    assert payload["pose"]["rotation"]["z"] == pytest.approx(0.0, abs=1e-12)
    assert payload["reset_kinematics"] is True
    actor_topic, actor_payload = connection.requests[-1]
    assert actor_topic == SET_OBJECT_POSE_TOPIC
    assert actor_payload["object_name"] == "Rover1"
    assert actor_payload["pose"] == payload["pose"]
    assert actor_payload["teleport"] is True
    rover.close()


def test_set_home_pose_replaces_the_return_home_target():
    rover, connection, _ = make_rover()
    connection.pose = (60.0, 5.0, 0.94, 90.0)
    # The constructor reads the spawn while the scene is paused; the session
    # replaces it with the pose measured from the first rendered frame.
    rover.set_home_pose(WorldPose(50.0, 0.0, 0.90, 0.0))
    home = rover.return_home()
    assert home.z == pytest.approx(0.90)
    _, payload = connection.requests[-2]
    assert payload["pose"]["translation"]["z"] == pytest.approx(0.90)
    rover.close()


def test_teleport_moves_the_rover_anywhere_and_resets_kinematics():
    rover, connection, _ = make_rover()
    rover.teleport(WorldPose(-300.0, 250.0, 0.9, 45.0))

    topic, payload = connection.requests[-2]
    assert topic == SET_POSE_TOPIC
    assert payload["pose"]["frame_id"] == "DEFAULT_FRAME"
    assert payload["pose"]["translation"] == {
        "x": pytest.approx(-300.0),
        "y": pytest.approx(250.0),
        "z": pytest.approx(0.9),
    }
    assert payload["reset_kinematics"] is True
    actor_topic, actor_payload = connection.requests[-1]
    assert actor_topic == SET_OBJECT_POSE_TOPIC
    assert actor_payload["object_name"] == "Rover1"
    assert actor_payload["pose"] == payload["pose"]
    assert actor_payload["teleport"] is True
    rover.close()


def test_teleport_to_the_current_pose_skips_the_actor_move():
    rover, connection, _ = make_rover()
    # Homing right after the vehicle was restored from its hidden spot asks
    # for the pose it is already at; the server refuses that actor move
    # ("Unable to move object"), so it must not be sent.
    connection.pose = (50.0, 0.0, 0.94, 0.0)
    rover.teleport(WorldPose(50.0, 0.0, 0.94, 0.0))
    topics = [topic for topic, _ in connection.requests]
    assert SET_POSE_TOPIC in topics
    assert SET_OBJECT_POSE_TOPIC not in topics
    rover.close()


def test_tracking_limits_are_derived_from_the_configured_envelope():
    configured = ConfiguredMotionLimits(
        max_speed_mps=5.0,
        max_accel_mps2=1.2,
        max_yaw_rate_dps=45.0,
        max_yaw_accel_dps2=60.0,
        control_hz=20.0,
        min_duration_s=0.5,
    )
    rover, _, _ = make_rover(motion=configured)
    limits = rover.tracking_limits
    assert limits.max_acceleration_mps2 == pytest.approx(1.2)
    assert limits.max_yaw_rate_deg_s == pytest.approx(45.0)
    assert limits.max_yaw_acceleration_deg_s2 == pytest.approx(60.0)
    assert limits.control_hz == pytest.approx(20.0)
    assert limits.min_duration_s == pytest.approx(0.5)
    rover.close()


def test_close_unsubscribes_camera_and_collision_topics():
    rover, connection, _ = make_rover()
    rover.close()
    topics = []
    for group in connection.client.unsubscribed:
        topics.extend([group] if isinstance(group, str) else group)
    assert COLLISION_TOPIC in topics
    assert f"{ROBOT_TOPIC}/sensors/FrontCamera/scene_camera" in topics