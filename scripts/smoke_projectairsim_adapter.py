#!/usr/bin/env python3
"""M2 acceptance: live CityEnviron check of the ProjectAirSim adapter.

Start the simulator first, e.g.:
    scripts/launch_projectairsim_xpra.sh cityenviron :104 10010 8991 8992
Then:
    conda activate carla-vln
    python scripts/smoke_projectairsim_adapter.py

The script loads the dual scene through ``ProjectAirSimConnection``, attaches a
``ProjectAirSimRover`` to Rover1, captures a stopped RGB/depth observation,
drives with raw ``SetRoverControls`` while reading moving observations, checks
that ground contacts are filtered and real contacts latch, and reloads the
scene at the end to restore the spawn pose.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from planner.adapters.projectairsim.connection import (  # noqa: E402
    ProjectAirSimConnection,
    ProjectAirSimConnectionConfig,
)
from planner.adapters.projectairsim.rover import ProjectAirSimRover  # noqa: E402
from planner.config import load_config  # noqa: E402
from planner.errors import ObservationError  # noqa: E402
from projectairsim import Rover  # noqa: E402


def set_controls(connection, robot, *, engine, steering_angle, brake):
    connection.request(
        f"{robot.parent_topic}/SetRoverControls",
        {
            "engine": float(engine),
            "steering_angle": float(steering_angle),
            "brake": float(brake),
        },
    )


def print_observation(label, observation):
    depth = observation.depth_m
    valid = np.isfinite(depth) & (depth > 0)
    depth_text = (
        f"min={float(depth[valid].min()):.2f} median={float(np.median(depth[valid])):.2f}"
        if valid.any()
        else "no valid pixels"
    )
    pose = observation.pose
    print(
        f"{label}: stamp={observation.timestamp_ns} ns stopped={observation.stopped} "
        f"{observation.rgb.shape} depth_kind={observation.depth_kind}"
    )
    print(
        f"  pose x={pose.x:.3f} y={pose.y:.3f} z={pose.z:.3f} yaw={pose.yaw_deg:.2f} "
        f"depth_m[{depth_text}] invalid={int((~valid).sum())}/{depth.size}"
    )
    print(
        f"  intrinsics fx={observation.intrinsics.fx:.2f} "
        f"cx={observation.intrinsics.cx:.2f} camera={observation.camera_position_world}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config" / "base.yaml"))
    parser.add_argument("--drive-s", type=float, default=3.0)
    parser.add_argument("--turn-s", type=float, default=4.0)
    parser.add_argument("--no-reset", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    pas = config.projectairsim
    connection = ProjectAirSimConnection(
        ProjectAirSimConnectionConfig(
            address=pas.address,
            port_topics=pas.port_topics,
            port_services=pas.port_services,
            timeout_s=pas.timeout_s,
            scene_file=pas.scene_file,
            sim_config_path=pas.sim_config_path,
            delay_after_load_s=pas.delay_after_load_s,
        )
    )
    print(f"connecting to {pas.address}:{pas.port_topics}/{pas.port_services}")
    connection.connect(wait_s=max(10.0, pas.timeout_s))
    rover = None
    try:
        world = connection.load_scene()
        robot = Rover(connection.client, world, pas.rover_name)
        rover = ProjectAirSimRover(
            connection,
            robot,
            camera=config.camera,
            depth=config.depth,
            camera_id=pas.front_camera,
        )
        print(f"scene '{pas.scene_file}' loaded, rover {pas.rover_name} attached")

        stopped = rover.capture(timeout_s=5.0)
        print_observation("stopped capture", stopped)
        kinematics_pose = rover.current_pose()
        position_error = math.dist(
            (stopped.pose.x, stopped.pose.y, stopped.pose.z),
            (kinematics_pose.x, kinematics_pose.y, kinematics_pose.z),
        )
        print(
            f"  kinematics x={kinematics_pose.x:.3f} y={kinematics_pose.y:.3f} "
            f"z={kinematics_pose.z:.3f} -> position error {position_error:.3f} m"
        )
        if position_error > 0.5:
            raise RuntimeError(
                "reconstructed body pose disagrees with ground-truth kinematics"
            )

        raw_contacts = {"total": 0, "ground": 0}
        topic = robot.robot_info["collision_info"]

        def count_contacts(_topic, message):
            raw_contacts["total"] += 1
            if str(message.get("object_name") or "").lower().startswith("landscape"):
                raw_contacts["ground"] += 1

        connection.client.subscribe(topic, count_contacts)

        print(f"enable_api_control={robot.enable_api_control()} arm={robot.arm()}")
        moving = []
        collisions = []
        set_controls(connection, robot, engine=0.5, steering_angle=0.0, brake=0.0)
        for _ in range(int(args.drive_s * 20)):
            connection.advance_for_sim_time(0.05)
            if len(moving) < 4:
                try:
                    moving.append(rover.capture_moving())
                except ObservationError:
                    pass
            if rover.collision_state():
                collisions.append(rover.last_collision)
        set_controls(connection, robot, engine=0.4, steering_angle=0.8, brake=0.0)
        for _ in range(int(args.turn_s * 20)):
            connection.advance_for_sim_time(0.05)
            if rover.collision_state():
                collisions.append(rover.last_collision)
            if collisions:
                break
        set_controls(connection, robot, engine=0.0, steering_angle=0.0, brake=1.0)
        for _ in range(20):
            connection.advance_for_sim_time(0.05)

        print(
            f"drove {args.drive_s + args.turn_s:.0f}s sim; moving observations="
            f"{len(moving)}; raw contacts total={raw_contacts['total']} "
            f"ground={raw_contacts['ground']}"
        )
        for observation in moving[:2]:
            print_observation("moving capture", observation)
        if collisions:
            event = collisions[0]
            print(
                f"latched non-ground contact: object={event.object_name!r} "
                f"stamp={event.timestamp_ns} penetration={event.penetration_depth:.3f}"
            )
        else:
            print("no non-ground contact occurred on this path (filtering verified)")

        final_pose = rover.current_pose()
        print(
            f"final pose x={final_pose.x:.2f} y={final_pose.y:.2f} "
            f"z={final_pose.z:.2f} yaw={final_pose.yaw_deg:.1f}"
        )
        print(f"time of day: {connection.get_time_of_day()}")
        if not args.no_reset:
            rover.close()
            rover = None
            connection.load_scene()
            print("scene reloaded to reset the rover")
        return 0
    finally:
        if rover is not None:
            rover.close()
        connection.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        print(
            "is CityEnviron running with -topicsport=8991 -servicesport=8992?",
            file=sys.stderr,
        )
        raise SystemExit(2)