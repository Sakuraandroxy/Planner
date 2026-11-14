import asyncio
import logging
import math

import pytest
from projectairsim.drone import YawControlMode
from projectairsim.types import LandedState

from planner.adapters.projectairsim.drone import ProjectAirSimDrone
from planner.adapters.projectairsim.moving_source import BorrowedMovingSource
from planner.config import CameraSettings, DepthSettings
from planner.domain.pose import WorldPose
from planner.errors import ExecutionError
from planner.services.segment_motion import MotionLimits as ConfiguredMotionLimits

WIDTH, HEIGHT = 4, 3
GROUND_Z = 0.85
ROBOT_TOPIC = "/Sim/SceneTest/robots/Drone1"
COLLISION_TOPIC = f"{ROBOT_TOPIC}/collision_info"
KINEMATICS_TOPIC = f"{ROBOT_TOPIC}/GetGroundTruthKinematics"
WORLD_TOPIC = "/Sim/SceneTest"
CAMERA_TOPIC = f"{ROBOT_TOPIC}/sensors/FrontCamera/scene_camera"

CAMERA = CameraSettings(
    width=WIDTH,
    height=HEIGHT,
    hfov_deg=90.0,
    offset_m=(1.0, 0.0, 0.0),
    mount_rpy_deg=(0.0, 0.0, 0.0),
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))


def scene_configuration():
    return {
        "actors": [
            {
                "name": "Drone1",
                "robot-config": {
                    "sensors": [
                        {
                            "id": "FrontCamera",
                            "type": "camera",
                            "origin": {"xyz": "1.0 0.0 0.0", "rpy-deg": "0 0 0"},
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


def _cancel(future):
    if not future.done():
        future.cancel()


class FakeRobot:
    """Queued native commands resolved by simulated time, like ProjectAirSim."""

    name = "Drone1"
    parent_topic = ROBOT_TOPIC
    world_parent_topic = WORLD_TOPIC
    robot_info = {"collision_info": COLLISION_TOPIC}

    def __init__(self):
        self.calls = []
        self.x, self.y, self.z = 10.0, 5.0, GROUND_Z
        self.roll_deg, self.pitch_deg, self.yaw_deg = 0.0, 0.0, 0.0
        self.landed = int(LandedState.LANDED)
        self.takeoff_result = False
        self.takeoff_lands = False
        self.takeoff_climb_m = 1.4
        self.takeoff_duration_s = 1.0
        self.land_result = True
        self.land_reports_flying = False
        self.land_duration_s = 1.0
        self.velocity_result = True
        self.commands = []
        self.cancels = 0
        self._pending = None

    # -- lifecycle ---------------------------------------------------------
    def enable_api_control(self):
        self.calls.append("enable_api_control")
        return True

    def arm(self):
        self.calls.append("arm")
        return True

    def get_landed_state(self):
        return self.landed

    def cancel_last_task(self):
        self.cancels += 1
        pending, self._pending = self._pending, None
        if pending is not None and not pending["task"].done():
            pending["loop"].call_soon_threadsafe(_cancel, pending["task"])

    # -- native commands ---------------------------------------------------
    async def takeoff_async(self, *, timeout_sec=30):
        loop = asyncio.get_running_loop()
        task = loop.create_future()
        self._pending = {
            "kind": "takeoff",
            "task": task,
            "loop": loop,
            "remaining": float(self.takeoff_duration_s),
            "timeout_s": timeout_sec,
        }
        return task

    async def land_async(self, *, timeout_sec=30):
        loop = asyncio.get_running_loop()
        task = loop.create_future()
        self._pending = {
            "kind": "land",
            "task": task,
            "loop": loop,
            "remaining": float(self.land_duration_s),
            "timeout_s": timeout_sec,
        }
        return task

    async def move_by_velocity_async(
        self,
        vn,
        ve,
        vd,
        *,
        duration,
        yaw_control_mode,
        yaw_is_rate,
        yaw,
    ):
        loop = asyncio.get_running_loop()
        task = loop.create_future()
        self.commands.append(
            {
                "velocity": (float(vn), float(ve), float(vd)),
                "duration": float(duration),
                "yaw_control_mode": yaw_control_mode,
                "yaw_is_rate": yaw_is_rate,
                "yaw_rad": float(yaw),
            }
        )
        self._pending = {
            "kind": "velocity",
            "task": task,
            "loop": loop,
            "remaining": float(duration),
            "velocity": (float(vn), float(ve), float(vd)),
            "yaw_rate_deg_s": math.degrees(float(yaw)),
        }
        return task

    # -- simulated time ----------------------------------------------------
    def advance(self, delta_s):
        pending = self._pending
        if pending is None:
            return
        step = min(float(delta_s), pending["remaining"])
        pending["remaining"] -= step
        if pending["kind"] == "velocity":
            vx, vy, vz = pending["velocity"]
            self.x += vx * step
            self.y += vy * step
            self.z += vz * step
            self.yaw_deg += pending["yaw_rate_deg_s"] * step
        if pending["remaining"] > 1e-9:
            return
        self._pending = None
        task = pending["task"]
        if pending["kind"] == "takeoff":
            if not self.takeoff_lands:
                self.landed = int(LandedState.FLYING)
                self.z -= self.takeoff_climb_m
            if not task.done():
                task.set_result(self.takeoff_result)
        elif pending["kind"] == "land":
            self.z = GROUND_Z
            if not self.land_reports_flying:
                self.landed = int(LandedState.LANDED)
            if not task.done():
                task.set_result(self.land_result)
        else:
            if not task.done():
                task.set_result(self.velocity_result)

    def kinematics(self, time_ns):
        roll, pitch, yaw = (
            math.radians(value) / 2
            for value in (self.roll_deg, self.pitch_deg, self.yaw_deg)
        )
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return {
            "time_stamp": time_ns,
            "pose": {
                "position": {"x": self.x, "y": self.y, "z": self.z},
                "orientation": {
                    "w": cr * cp * cy + sr * sp * sy,
                    "x": sr * cp * cy - cr * sp * sy,
                    "y": cr * sp * cy + sr * cp * sy,
                    "z": cr * cp * sy - sr * sp * cy,
                },
            },
        }


class FakeConnection:
    def __init__(self, robot):
        self.client = FakeClient()
        self.robot = robot
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
            return self.robot.kinematics(self.time_ns)
        if topic.endswith("/SetPose"):
            pose = payload["pose"]
            translation = pose["translation"]
            self.robot.x = translation["x"]
            self.robot.y = translation["y"]
            self.robot.z = translation["z"]
            rotation = pose["rotation"]
            self.robot.yaw_deg = math.degrees(
                2.0 * math.atan2(rotation["z"], rotation["w"])
            )
        return {}

    def advance_for_sim_time(self, delta_s):
        self.advances.append(delta_s)
        self.time_ns += int(round(delta_s * 1e9))
        self.robot.advance(float(delta_s))


def make_drone(**robot_settings):
    robot = FakeRobot()
    for name, value in robot_settings.items():
        setattr(robot, name, value)
    connection = FakeConnection(robot)
    clock = FakeClock()
    drone = ProjectAirSimDrone(
        connection,
        robot,
        camera=CAMERA,
        depth=DepthSettings(),
        camera_id="FrontCamera",
        clock=clock,
        sleep=clock.sleep,
    )
    return drone, connection, robot, clock


def test_start_enables_api_control_and_arms():
    drone, _, robot, _ = make_drone()
    drone.start()
    assert robot.calls == ["enable_api_control", "arm"]
    drone.close()


def test_start_records_the_spawn_pose_before_the_drone_is_hidden():
    drone, _, robot, _ = make_drone()
    drone.start()
    home = drone._home_pose
    assert home is not None
    assert (home.x, home.y, home.z) == pytest.approx((10.0, 5.0, GROUND_Z))

    # Hiding an inactive vehicle is a far-away teleport; the recorded home
    # must stay at the spawn instead of following the drone out of view.
    drone.teleport(
        WorldPose(home.x - 300.0, home.y - 300.0, home.z, home.yaw_deg)
    )
    returned = drone.return_home()
    assert (returned.x, returned.y, returned.z) == pytest.approx(
        (home.x, home.y, home.z)
    )
    drone.close()


def test_prepare_tolerates_a_false_native_takeoff_with_measured_climb(caplog):
    drone, _, robot, _ = make_drone()
    drone.start()
    with caplog.at_level(
        logging.WARNING, logger="planner.adapters.projectairsim.drone"
    ):
        drone.prepare(takeoff_if_landed=True)
    assert robot.landed == int(LandedState.FLYING)
    # The first hold tick sees the takeoff as finite-difference velocity and
    # damps it; the reference tolerates the residual inside the position
    # tolerance, so only require the measured climb.
    assert GROUND_Z - robot.z >= robot.takeoff_climb_m
    assert "Native takeoff was not confirmed" in caplog.text
    assert drone._holding.active is True
    drone.close()


def test_prepare_rejects_a_takeoff_that_is_still_landed():
    drone, _, robot, _ = make_drone(takeoff_lands=True)
    drone.start()
    with pytest.raises(ExecutionError, match="still landed"):
        drone.prepare(takeoff_if_landed=True)
    drone.close()


def test_prepare_rejects_a_takeoff_without_measured_climb():
    # Clear the 0.1 m ground band so the landed-state check passes, but stay
    # inside the 0.3 m position tolerance that defines a confirmed climb.
    drone, _, robot, _ = make_drone(takeoff_climb_m=0.2)
    drone.start()
    with pytest.raises(ExecutionError, match="measured climb"):
        drone.prepare(takeoff_if_landed=True)
    drone.close()


def test_prepare_without_takeoff_holds_an_airborne_vehicle():
    drone, _, robot, _ = make_drone()
    drone.start()
    robot.landed = int(LandedState.FLYING)
    robot.z = -3.0
    robot.commands.clear()
    drone.prepare(takeoff_if_landed=False)
    assert drone._holding.active is True
    assert all(command["yaw_is_rate"] for command in robot.commands)
    drone.close()


def test_takeoff_runs_the_native_command():
    drone, _, robot, _ = make_drone(takeoff_result=True)
    drone.start()
    drone.takeoff()
    assert robot.landed == int(LandedState.FLYING)
    assert robot.z == pytest.approx(GROUND_Z - robot.takeoff_climb_m)
    drone.close()


def test_land_returns_the_measured_landed_state():
    drone, _, robot, _ = make_drone()
    drone.start()
    robot.landed = int(LandedState.FLYING)
    robot.z = -2.0
    assert drone.land() is True
    assert robot.landed == int(LandedState.LANDED)
    assert robot.z == pytest.approx(GROUND_Z)
    drone.close()


def test_land_confirms_touchdown_when_landed_state_lags():
    drone, _, robot, _ = make_drone(land_reports_flying=True)
    drone.start()
    drone.prepare(takeoff_if_landed=True)
    robot.z = -2.0
    assert drone.land() is True
    assert robot.landed == int(LandedState.FLYING)
    assert robot.z == pytest.approx(GROUND_Z)
    drone.close()


def test_prepare_takes_off_again_after_a_lagging_land():
    drone, _, robot, _ = make_drone(land_reports_flying=True)
    drone.start()
    drone.prepare(takeoff_if_landed=True)
    robot.z = -2.0
    assert drone.land() is True
    drone.prepare(takeoff_if_landed=True)
    assert robot.z < GROUND_Z - 0.5
    drone.close()


def test_stream_velocity_sends_move_by_velocity_and_advances_one_step():
    drone, connection, robot, _ = make_drone()
    drone.start()
    connection.requests.clear()
    connection.advances.clear()
    drone.stream_velocity((2.0, 0.0, 0.0), 30.0, 0.05)
    command = robot.commands[-1]
    assert command["velocity"] == (2.0, 0.0, 0.0)
    assert command["duration"] == pytest.approx(0.05)
    assert command["yaw_is_rate"] is True
    assert command["yaw_control_mode"] == YawControlMode.MaxDegreeOfFreedom
    assert command["yaw_rad"] == pytest.approx(math.radians(30.0))
    assert connection.advances == [0.05]
    assert robot.x == pytest.approx(10.0 + 2.0 * 0.05)
    drone.close()


def test_stream_velocity_reports_an_interrupted_command():
    drone, _, robot, _ = make_drone(velocity_result=False)
    drone.start()
    with pytest.raises(ExecutionError, match="interrupted"):
        drone.stream_velocity((1.0, 0.0, 0.0), 0.0, 0.05)
    drone.close()


def test_stream_velocity_requires_api_control():
    drone, _, _, _ = make_drone()
    with pytest.raises(ExecutionError, match="start"):
        drone.stream_velocity((1.0, 0.0, 0.0), 0.0, 0.05)
    drone.close()


def test_stream_velocity_rejects_bad_requests():
    drone, _, _, _ = make_drone()
    drone.start()
    with pytest.raises(ExecutionError):
        drone.stream_velocity((float("nan"), 0.0, 0.0), 0.0, 0.05)
    with pytest.raises(ExecutionError):
        drone.stream_velocity((1.0, 0.0, 0.0), 0.0, 0.0)
    with pytest.raises(ExecutionError):
        drone.stream_velocity((1.0, 0.0), 0.0, 0.05)
    drone.close()


def test_stream_velocity_releases_the_position_hold():
    drone, _, robot, _ = make_drone()
    drone.start()
    robot.landed = int(LandedState.FLYING)
    robot.z = -3.0
    drone.hover()
    assert drone._holding.active is True
    drone.stream_velocity((0.5, 0.0, 0.0), 0.0, 0.05)
    assert drone._holding.active is False
    drone.close()


def test_hover_starts_the_hold_and_commands_one_period():
    drone, connection, robot, _ = make_drone()
    drone.start()
    robot.landed = int(LandedState.FLYING)
    robot.z = -3.0
    connection.advances.clear()
    drone.hover()
    assert drone._holding.active is True
    assert robot.commands[-1]["duration"] == pytest.approx(1 / 20.0)
    assert connection.advances == [pytest.approx(1 / 20.0)]
    drone.close()


def test_hover_requires_api_control():
    drone, _, _, _ = make_drone()
    with pytest.raises(ExecutionError, match="start"):
        drone.hover()
    drone.close()


def test_idle_tick_advances_the_paused_world_when_landed():
    drone, connection, _, _ = make_drone()
    drone.start()
    connection.advances.clear()
    drone.idle_tick()
    assert connection.advances == [pytest.approx(1 / 20.0)]
    drone.close()


def test_idle_tick_keeps_an_active_hold_alive():
    drone, connection, robot, clock = make_drone()
    drone.start()
    robot.landed = int(LandedState.FLYING)
    robot.z = -3.0
    drone.hover()
    connection.advances.clear()
    robot.commands.clear()
    # The hold ticks at the control rate; idle polling must not run ahead.
    clock.sleep(1 / 20.0)
    drone.idle_tick()
    assert robot.commands[-1]["duration"] == pytest.approx(1 / 20.0)
    assert connection.advances == [pytest.approx(1 / 20.0)]
    drone.close()


def test_execute_segment_requires_api_control():
    drone, _, _, _ = make_drone()
    with pytest.raises(ExecutionError, match="start"):
        drone.execute_segment(None)
    drone.close()


def test_stream_state_reports_attitude_and_finite_difference_velocity():
    drone, connection, robot, _ = make_drone()
    drone.start()
    robot.roll_deg, robot.pitch_deg, robot.yaw_deg = 10.0, -5.0, 45.0
    state = drone.stream_state()
    assert state.pose.yaw_deg == pytest.approx(45.0)
    assert state.pitch_deg == pytest.approx(-5.0)
    assert state.roll_deg == pytest.approx(10.0)
    assert state.velocity_world == (0.0, 0.0, 0.0)
    connection.advance_for_sim_time(1.0)
    robot.x += 1.0
    state = drone.stream_state()
    assert state.velocity_world[0] == pytest.approx(1.0)
    drone.close()


def test_cancel_sets_the_flag_and_cancels_the_native_task():
    drone, _, robot, _ = make_drone()
    drone.start()
    drone.cancel()
    assert robot.cancels == 1
    drone.close()


def test_collision_state_latches_one_non_ground_contact():
    drone, connection, _, _ = make_drone()
    callback = connection.client.callbacks[COLLISION_TOPIC]
    callback(None, {"object_name": "Landscape_1"})
    assert drone.collision_state() is False
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
    assert drone.collision_state() is True
    assert drone.collision_state() is False
    assert drone.last_collision.object_name == "Building_3"
    assert drone.last_collision.timestamp_ns == 500
    drone.close()


def test_tracking_limits_are_derived_from_the_configured_envelope():
    configured = ConfiguredMotionLimits(
        max_speed_mps=5.0,
        max_accel_mps2=1.2,
        max_yaw_rate_dps=45.0,
        max_yaw_accel_dps2=60.0,
        control_hz=20.0,
        min_duration_s=0.5,
    )
    robot = FakeRobot()
    clock = FakeClock()
    configured_drone = ProjectAirSimDrone(
        FakeConnection(robot),
        robot,
        camera=CAMERA,
        motion=configured,
        clock=clock,
        sleep=clock.sleep,
    )
    limits = configured_drone.tracking_limits
    assert limits.max_acceleration_mps2 == pytest.approx(1.2)
    assert limits.max_yaw_rate_deg_s == pytest.approx(45.0)
    assert limits.max_yaw_acceleration_deg_s2 == pytest.approx(60.0)
    assert limits.control_hz == pytest.approx(20.0)
    assert limits.min_duration_s == pytest.approx(0.5)
    configured_drone.close()


def test_borrowed_moving_source_delegates_and_keeps_the_vehicle_open():
    class Vehicle:
        def __init__(self):
            self.capture_budgets = []
            self.budgets = []
            self.closed = False

        def capture(self, *, budget=None):
            self.capture_budgets.append(budget)
            return "stopped-frame"

        def capture_moving(self, *, budget=None):
            self.budgets.append(budget)
            return "frame"

        def close(self):
            self.closed = True

    vehicle = Vehicle()
    source = BorrowedMovingSource(vehicle)
    assert source.capture(budget="stop-budget") == "stopped-frame"
    assert vehicle.capture_budgets == ["stop-budget"]
    assert source.capture_moving(budget="budget") == "frame"
    assert vehicle.budgets == ["budget"]
    source.close()
    assert vehicle.closed is False


def test_close_unsubscribes_camera_and_collision_topics():
    drone, connection, _, _ = make_drone()
    drone.close()
    topics = []
    for group in connection.client.unsubscribed:
        topics.extend([group] if isinstance(group, str) else group)
    assert COLLISION_TOPIC in topics
    assert CAMERA_TOPIC in topics

def test_return_home_teleports_to_the_spawn_pose_and_confirms_touchdown():
    drone, connection, robot, _ = make_drone()
    drone.start()
    drone.prepare(takeoff_if_landed=True)
    assert drone._home_pose is not None
    assert drone._home_pose.x == pytest.approx(10.0)
    assert drone._home_pose.y == pytest.approx(5.0)
    assert drone._home_pose.z == pytest.approx(GROUND_Z)
    # The native controller keeps reporting FLYING after the spawn descent;
    # the measured altitude at the captured ground reference is authoritative.
    assert robot.landed == int(LandedState.FLYING)

    pose = drone.return_home()
    assert (pose.x, pose.y, pose.z) == pytest.approx((10.0, 5.0, GROUND_Z))
    assert (robot.x, robot.y, robot.z) == pytest.approx((10.0, 5.0, GROUND_Z))
    assert pose.yaw_deg == pytest.approx(0.0, abs=1e-6)
    assert robot.landed == int(LandedState.FLYING)
    assert drone._is_landed()

    set_pose = [
        payload
        for topic, payload in connection.requests
        if topic.endswith("/SetPose")
    ][-1]
    assert set_pose["reset_kinematics"] is True
    assert set_pose["pose"]["frame_id"] == "DEFAULT_FRAME"
    assert set_pose["pose"]["translation"] == {
        "x": pytest.approx(10.0),
        "y": pytest.approx(5.0),
        "z": pytest.approx(GROUND_Z),
    }
    assert set_pose["pose"]["rotation"]["w"] == pytest.approx(1.0)
    assert set_pose["pose"]["rotation"]["z"] == pytest.approx(0.0)
    drone.close()


def test_teleport_to_the_current_pose_skips_the_actor_move():
    drone, connection, _, _ = make_drone()
    drone.start()
    # Homing right after the drone was restored from its hidden spot asks
    # for the pose it is already at; the server refuses that actor move
    # ("Unable to move object"), so it must not be sent.
    drone.teleport(WorldPose(10.0, 5.0, GROUND_Z, 0.0))
    topics = [topic for topic, _ in connection.requests]
    assert f"{ROBOT_TOPIC}/SetPose" in topics
    assert f"{WORLD_TOPIC}/SetObjectPose" not in topics
    drone.close()


def test_teleport_moves_the_drone_without_a_touchdown_sequence():
    drone, connection, robot, _ = make_drone()
    drone.start()
    drone.teleport(WorldPose(-300.0, -295.0, GROUND_Z, 45.0))

    assert (robot.x, robot.y, robot.z) == pytest.approx(
        (-300.0, -295.0, GROUND_Z)
    )
    payload = [
        item
        for topic, item in connection.requests
        if topic.endswith("/SetPose")
    ][-1]
    assert payload["reset_kinematics"] is True
    assert payload["pose"]["frame_id"] == "DEFAULT_FRAME"
    actor_topic, actor_payload = [
        (topic, item)
        for topic, item in connection.requests
        if topic.endswith("/SetObjectPose")
    ][-1]
    assert actor_topic == f"{WORLD_TOPIC}/SetObjectPose"
    assert actor_payload["object_name"] == "Drone1"
    assert actor_payload["pose"] == payload["pose"]
    assert actor_payload["teleport"] is True
    drone.close()
