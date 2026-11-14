import math
from types import SimpleNamespace
import time

import numpy as np
import pytest

from planner.adapters.projectairsim.collision import CollisionMonitor
from planner.adapters.projectairsim.frames import CameraTopicStream
from planner.adapters.projectairsim.observation import ProjectAirSimObservationSource
from planner.config import CameraSettings, DepthSettings
from planner.domain.execution import ExecutionBudget
from planner.errors import BudgetExceeded, ObservationError

WIDTH, HEIGHT = 4, 3
ROBOT_TOPIC = "/Sim/SceneTest/robots/Rover1"
RGB_TOPIC = f"{ROBOT_TOPIC}/sensors/FrontCamera/scene_camera"
DEPTH_TOPIC = f"{ROBOT_TOPIC}/sensors/FrontCamera/depth_camera"
COLLISION_TOPIC = f"{ROBOT_TOPIC}/collision_info"

CAMERA = CameraSettings(
    width=WIDTH,
    height=HEIGHT,
    hfov_deg=90.0,
    offset_m=(1.8, 0.0, 0.2),
    mount_rpy_deg=(0.0, -3.0, 0.0),
)
BODY_POSITION = (50.0, 0.0, 0.94)
CAMERA_POSITION = (51.8, 0.0, 1.14)
# rpy "0 -3 0" as the simulator published it.
MOUNT_QUATERNION = (0.9996573328971863, 0.0, -0.026176949962973595, 0.0)


def scene_configuration(width=WIDTH, height=HEIGHT, fov=90):
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


def bgr_image():
    return np.full((HEIGHT, WIDTH, 3), (10, 20, 30), dtype=np.uint8)


def scene_message(
    stamp,
    bgr=None,
    *,
    position=CAMERA_POSITION,
    quaternion=MOUNT_QUATERNION,
    encoding="BGR",
):
    image = bgr_image() if bgr is None else bgr
    return {
        "encoding": encoding,
        "width": WIDTH,
        "height": HEIGHT,
        "time_stamp": stamp,
        "data": image.tobytes(),
        "pos_x": position[0],
        "pos_y": position[1],
        "pos_z": position[2],
        "rot_w": quaternion[0],
        "rot_x": quaternion[1],
        "rot_y": quaternion[2],
        "rot_z": quaternion[3],
    }


def depth_message(
    stamp,
    depth=None,
    *,
    position=CAMERA_POSITION,
    quaternion=MOUNT_QUATERNION,
):
    image = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float16) if depth is None else depth
    return {
        "encoding": "16FC1",
        "width": WIDTH,
        "height": HEIGHT,
        "time_stamp": stamp,
        "data": np.asarray(image, dtype=np.float16).tobytes(),
        "pos_x": position[0],
        "pos_y": position[1],
        "pos_z": position[2],
        "rot_w": quaternion[0],
        "rot_x": quaternion[1],
        "rot_y": quaternion[2],
        "rot_z": quaternion[3],
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
    def __init__(self, scene=None):
        self.client = FakeClient()
        self._scene = scene_configuration() if scene is None else scene
        self.time_ns = 1_000_000_000
        self.advance_hook = None

    def scene_configuration(self):
        return self._scene

    def sim_time_ns(self):
        return self.time_ns

    def advance_for_sim_time(self, delta_s):
        self.time_ns += int(round(delta_s * 1e9))
        if self.advance_hook is not None:
            self.advance_hook(self.time_ns)


def make_stream(frame_sink=None):
    client = FakeClient()
    stream = CameraTopicStream(
        client,
        rgb_topic=RGB_TOPIC,
        depth_topic=DEPTH_TOPIC,
        width=WIDTH,
        height=HEIGHT,
        frame_sink=frame_sink,
    )
    stream.start()
    return stream, client


def feed(client, *, stamp, bgr=None, depth=None, **kwargs):
    client.callbacks[RGB_TOPIC](None, scene_message(stamp, bgr, **kwargs))
    client.callbacks[DEPTH_TOPIC](None, depth_message(stamp, depth, **kwargs))


# -- CameraTopicStream -----------------------------------------------------
def test_stream_pairs_only_identical_capture_timestamps():
    stream, client = make_stream()
    client.callbacks[RGB_TOPIC](None, scene_message(100))
    client.callbacks[DEPTH_TOPIC](None, depth_message(101))
    assert stream.poll_next(0) is None
    client.callbacks[DEPTH_TOPIC](None, depth_message(100))
    pair = stream.poll_next(0)
    assert pair is not None and pair.timestamp_ns == 100


def test_stream_poll_next_is_ordered_and_prunes_older_frames():
    stream, client = make_stream()
    feed(client, stamp=200)
    feed(client, stamp=300)
    assert stream.poll_next(150).timestamp_ns == 200
    assert stream.poll_next(150).timestamp_ns == 300
    assert stream.poll_next(150) is None


def test_stream_latest_pair_discards_unread_frames():
    stream, client = make_stream()
    feed(client, stamp=200)
    feed(client, stamp=300)
    assert stream.latest_pair().timestamp_ns == 300
    assert stream.poll_next(0) is None


def test_stream_publishes_bgra_to_the_frame_sink():
    published = []
    stream, client = make_stream(lambda array, timestamp: published.append((array, timestamp)))
    feed(client, stamp=123)
    assert len(published) == 1
    image, timestamp = published[0]
    assert image.shape == (HEIGHT, WIDTH, 4)
    assert timestamp == pytest.approx(123e-9)
    assert np.all(image[..., :3] == (10, 20, 30))
    assert np.all(image[..., 3] == 255)


def test_stream_callback_errors_surface_on_poll():
    stream, client = make_stream()
    client.callbacks[RGB_TOPIC](None, scene_message(100, encoding="RGB "))
    with pytest.raises(ObservationError, match="camera topic stream failed"):
        stream.poll_next(0)


def test_stream_close_unsubscribes_both_topics():
    stream, client = make_stream()
    stream.close()
    assert client.unsubscribed == [[RGB_TOPIC, DEPTH_TOPIC]]
    with pytest.raises(ObservationError, match="closed"):
        stream.poll_next(0)


# -- ProjectAirSimObservationSource ---------------------------------------
def make_observation_source(scene=None):
    connection = FakeConnection(scene)
    robot = SimpleNamespace(name="Rover1", parent_topic=ROBOT_TOPIC)
    source = ProjectAirSimObservationSource(
        connection,
        robot,
        camera=CAMERA,
        depth=DepthSettings(),
        camera_id="FrontCamera",
    )
    return source, connection


def test_stopped_capture_steps_until_a_fresh_pair_and_decodes_it():
    source, connection = make_observation_source()
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float16)
    depth[0, 0] = np.inf
    depth[0, 1] = np.float16(65504.0)
    connection.advance_hook = lambda stamp: feed(
        connection.client, stamp=stamp, depth=depth
    )
    observation = source.capture(timeout_s=1.0)
    assert observation.stopped is True
    assert observation.timestamp_ns > 1_000_000_000
    assert observation.depth_kind == "ray_range"
    assert observation.intrinsics.width == WIDTH
    assert observation.depth_intrinsics is observation.intrinsics
    # BGR message channels are reordered to RGB.
    assert tuple(observation.rgb[1, 1]) == (30, 20, 10)
    # Sky sentinels become NaN; metric values pass through.
    assert math.isnan(float(observation.depth_m[0, 0]))
    assert math.isnan(float(observation.depth_m[0, 1]))
    assert observation.depth_m[1, 0] == pytest.approx(2.0)
    assert observation.depth_image.shape == (HEIGHT, WIDTH, 3)
    # The body pose is recovered from the camera pose and the configured mount.
    assert observation.pose.as_tuple() == pytest.approx(
        (*BODY_POSITION, 0.0), abs=1e-5
    )
    assert observation.camera_position_world == pytest.approx(CAMERA_POSITION)
    assert np.allclose(
        observation.camera_matrix[:3, 3], CAMERA_POSITION
    )


def test_moving_capture_reads_the_latest_pair_without_stepping():
    source, connection = make_observation_source()
    feed(connection.client, stamp=1_500_000_000)
    feed(connection.client, stamp=1_600_000_000)
    observation = source.capture_moving()
    assert observation.stopped is False
    assert observation.timestamp_ns == 1_600_000_000
    assert connection.time_ns == 1_000_000_000  # no world advance


def test_stopped_capture_respects_an_expired_budget():
    source, _ = make_observation_source()
    budget = ExecutionBudget(deadline_monotonic=time.monotonic() - 1.0)
    with pytest.raises(BudgetExceeded):
        source.capture(budget=budget)


def test_stopped_capture_times_out_without_frames():
    source, _ = make_observation_source()
    with pytest.raises(ObservationError, match="no synchronized"):
        source.capture(timeout_s=0.01, step_s=0.001)


def test_unregistered_scene_and_depth_poses_are_rejected():
    source, connection = make_observation_source()
    connection.advance_hook = lambda stamp: (
        connection.client.callbacks[RGB_TOPIC](None, scene_message(stamp)),
        connection.client.callbacks[DEPTH_TOPIC](
            None, depth_message(stamp, position=(60.0, 0.0, 1.14))
        ),
    )
    with pytest.raises(ObservationError, match="not registered"):
        source.capture(timeout_s=1.0)


def test_camera_topics_follow_the_robot_parent_topic():
    source, connection = make_observation_source()
    assert RGB_TOPIC in connection.client.callbacks
    assert DEPTH_TOPIC in connection.client.callbacks
    source.close()


def test_observation_source_rejects_a_robot_without_parent_topic():
    connection = FakeConnection()
    with pytest.raises(ValueError, match="parent topic"):
        ProjectAirSimObservationSource(
            connection,
            SimpleNamespace(name="Rover1"),
            camera=CAMERA,
            depth=DepthSettings(),
            camera_id="FrontCamera",
        )


def test_observation_source_rejects_a_scene_without_depth_capture():
    scene = scene_configuration()
    sensor = scene["actors"][0]["robot-config"]["sensors"][0]
    sensor["capture-settings"] = sensor["capture-settings"][:1]
    connection = FakeConnection(scene=scene)
    with pytest.raises(ValueError, match="depth camera"):
        ProjectAirSimObservationSource(
            connection,
            SimpleNamespace(name="Rover1", parent_topic=ROBOT_TOPIC),
            camera=CAMERA,
            depth=DepthSettings(),
            camera_id="FrontCamera",
        )


def test_observation_source_rejects_config_scene_camera_mismatch():
    connection = FakeConnection(scene=scene_configuration(width=640))
    robot = SimpleNamespace(name="Rover1", parent_topic=ROBOT_TOPIC)
    with pytest.raises(ValueError, match="width"):
        ProjectAirSimObservationSource(
            connection,
            robot,
            camera=CAMERA,
            depth=DepthSettings(),
            camera_id="FrontCamera",
        )


# -- CollisionMonitor ------------------------------------------------------
def collision_message(name, stamp=123, penetration=0.0):
    return {
        "object_name": name,
        "time_stamp": stamp,
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "normal": {"x": 0.0, "y": 0.0, "z": -1.0},
        "penetration_depth": penetration,
    }


def test_ground_contacts_are_ignored():
    client = FakeClient()
    monitor = CollisionMonitor(client, COLLISION_TOPIC)
    monitor.start()
    client.callbacks[COLLISION_TOPIC](None, collision_message("Landscape_1"))
    client.callbacks[COLLISION_TOPIC](None, collision_message("landscape_2"))
    assert monitor.consume() is None


def test_neighborhood_road_surface_contacts_are_ignored():
    client = FakeClient()
    monitor = CollisionMonitor(client, COLLISION_TOPIC)
    monitor.start()
    client.callbacks[COLLISION_TOPIC](None, collision_message("Road_89"))
    client.callbacks[COLLISION_TOPIC](None, collision_message("road_82"))
    assert monitor.consume() is None


def test_self_contacts_are_ignored_by_exact_name():
    client = FakeClient()
    monitor = CollisionMonitor(
        client, COLLISION_TOPIC, ignored_names=("Rover1",)
    )
    monitor.start()
    client.callbacks[COLLISION_TOPIC](None, collision_message("Rover1"))
    assert monitor.consume() is None
    client.callbacks[COLLISION_TOPIC](None, collision_message("Rover10"))
    assert monitor.consume() is not None


def test_real_contacts_are_latched_once_and_consumed():
    client = FakeClient()
    monitor = CollisionMonitor(client, COLLISION_TOPIC)
    monitor.start()
    client.callbacks[COLLISION_TOPIC](None, collision_message("Building_3", 500, 0.2))
    client.callbacks[COLLISION_TOPIC](None, collision_message("Tree_9", 501))
    event = monitor.consume()
    assert event is not None
    assert event.object_name == "Building_3"
    assert event.timestamp_ns == 500
    assert event.position == (1.0, 2.0, 3.0)
    assert event.normal == (0.0, 0.0, -1.0)
    assert event.penetration_depth == pytest.approx(0.2)
    assert monitor.consume() is None


def test_malformed_collision_messages_are_ignored():
    client = FakeClient()
    monitor = CollisionMonitor(client, COLLISION_TOPIC)
    monitor.start()
    client.callbacks[COLLISION_TOPIC](None, {"object_name": None})
    client.callbacks[COLLISION_TOPIC](None, {})
    assert monitor.consume() is None


def test_collision_monitor_accepts_custom_ignored_prefixes():
    client = FakeClient()
    monitor = CollisionMonitor(
        client, COLLISION_TOPIC, ignored_prefixes=("Landscape", "Curb")
    )
    monitor.start()
    client.callbacks[COLLISION_TOPIC](None, collision_message("Curb_01"))
    assert monitor.consume() is None
    client.callbacks[COLLISION_TOPIC](None, collision_message("Barrier_1"))
    assert monitor.consume() is not None


def test_collision_monitor_close_drops_pending_and_unsubscribes():
    client = FakeClient()
    monitor = CollisionMonitor(client, COLLISION_TOPIC)
    monitor.start()
    client.callbacks[COLLISION_TOPIC](None, collision_message("Building_3"))
    monitor.close()
    assert monitor.consume() is None
    assert client.unsubscribed == [COLLISION_TOPIC]