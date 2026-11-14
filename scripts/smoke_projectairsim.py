#!/usr/bin/env python3
"""M1 smoke test: load the dual-vehicle ProjectAirSim scene and capture frames.

Start the simulator first, e.g.:
    scripts/launch_projectairsim_xpra.sh cityenviron :104 10010 8991 8992
Then:
    conda activate carla-vln
    python scripts/smoke_projectairsim.py

The script loads scene_city_dual.jsonc (Drone1 + Rover1), pauses the clock,
steps until the Rover FrontCamera RGB/depth pair and the Drone Chase frame
arrive, prints their statistics and writes them under output/smoke_projectairsim.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from planner.config import load_config  # noqa: E402
from projectairsim import Drone, ProjectAirSimClient, Rover, World  # noqa: E402
from projectairsim.utils import unpack_image  # noqa: E402


def resolve(path: str) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


def depth_preview(depth_m: np.ndarray, far_m: float = 50.0) -> np.ndarray:
    """Temporary smoke preview: linear grey, near = bright."""
    finite = np.nan_to_num(depth_m, nan=far_m, posinf=far_m)
    normalized = np.clip(finite / far_m, 0.0, 1.0)
    return ((1.0 - normalized) * 255.0).astype(np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config" / "base.yaml"))
    parser.add_argument("--out", default=str(ROOT / "output" / "smoke_projectairsim"))
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    config = load_config(args.config)
    pas = config.projectairsim
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    client = ProjectAirSimClient(pas.address, pas.port_topics, pas.port_services)
    client.connect()
    print(f"connected to ProjectAirSim at {pas.address}:"
          f"{pas.port_topics}/{pas.port_services}")
    try:
        world = World(
            client,
            pas.scene_file,
            delay_after_load_sec=pas.delay_after_load_s,
            sim_config_path=str(resolve(pas.sim_config_path)),
        )
        rover = Rover(client, world, pas.rover_name)
        drone = Drone(client, world, pas.drone_name)
        world.pause()
        print(f"scene '{pas.scene_file}' loaded; clock paused at "
              f"{world.get_sim_time()} ns")

        frames: dict[str, list[dict]] = {"rover_rgb": [], "rover_depth": [], "drone_rgb": []}

        def collect(name):
            def callback(_topic, message):
                frames[name].append(message)
            return callback

        rover_base = f"{rover.parent_topic}/sensors/{pas.front_camera}"
        drone_base = f"{drone.parent_topic}/sensors/{pas.chase_camera}"
        topics = {
            "rover_rgb": f"{rover_base}/scene_camera",
            "rover_depth": f"{rover_base}/depth_camera",
            "drone_rgb": f"{drone_base}/scene_camera",
        }
        for name, topic in topics.items():
            client.subscribe(topic, collect(name))
            print(f"subscribed {name}: {topic}")

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            world.continue_for_sim_time(50_000_000, wait_until_complete=True)
            if all(frames[name] for name in frames):
                break
            time.sleep(0.02)
        missing = [name for name, messages in frames.items() if not messages]
        if missing:
            raise TimeoutError(f"no frames within {args.timeout:.0f}s: {missing}")
        print(f"sim time {world.get_sim_time()} ns, frame counts "
              f"{ {name: len(messages) for name, messages in frames.items()} }")

        rover_kinematics = rover.get_ground_truth_kinematics()
        drone_kinematics = drone.get_ground_truth_kinematics()
        for label, kinematics in (("rover", rover_kinematics), ("drone", drone_kinematics)):
            pose = kinematics["pose"]
            position = pose["position"]
            print(f"{label} pose: x={position['x']:.2f} y={position['y']:.2f} "
                  f"z={position['z']:.2f} quat="
                  f"({pose['orientation']['w']:.4f}, {pose['orientation']['x']:.4f}, "
                  f"{pose['orientation']['y']:.4f}, {pose['orientation']['z']:.4f})")

        rgb_message = frames["rover_rgb"][-1]
        depth_message = frames["rover_depth"][-1]
        print(f"rover frame stamp={rgb_message['time_stamp']} ns "
              f"{rgb_message['width']}x{rgb_message['height']} "
              f"encoding={rgb_message['encoding']}; depth stamp="
              f"{depth_message['time_stamp']} ns encoding={depth_message['encoding']}")
        print(f"rover camera pose: ({rgb_message['pos_x']:.3f}, "
              f"{rgb_message['pos_y']:.3f}, {rgb_message['pos_z']:.3f})")

        rgb = unpack_image(rgb_message)
        depth = unpack_image(depth_message).astype(np.float32)
        valid = np.isfinite(depth) & (depth < 60000.0)
        if valid.any():
            print(f"rover depth m (ray range): min={float(depth[valid].min()):.2f} "
                  f"median={float(np.median(depth[valid])):.2f} "
                  f"max={float(depth[valid].max()):.2f} "
                  f"invalid={int((~valid).sum())}/{depth.size}")
        else:
            print("rover depth: no valid pixels")
        cv2.imwrite(str(out / "rover_front_rgb.png"), rgb)
        cv2.imwrite(str(out / "rover_front_depth.png"), depth_preview(depth))
        np.save(out / "rover_front_depth_m.npy", depth)

        drone_rgb = unpack_image(frames["drone_rgb"][-1])
        cv2.imwrite(str(out / "drone_chase_rgb.png"), drone_rgb)
        print(f"drone chase frame {drone_rgb.shape} mean={float(drone_rgb.mean()):.1f}")

        print(f"artifacts written to {out}")
        return 0
    finally:
        client.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except TimeoutError as error:
        print(f"error: {error}", file=sys.stderr)
        print("is CityEnviron running with -topicsport=8991 -servicesport=8992?",
              file=sys.stderr)
        raise SystemExit(2)