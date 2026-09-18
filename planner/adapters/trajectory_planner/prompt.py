from __future__ import annotations

import json

from planner.domain.observation import Observation


def build_prompt(instruction: str, observation: Observation) -> str:
    camera = {
        "camera_id": observation.camera_id,
        "horizontal_fov_deg": round(observation.intrinsics.horizontal_fov_deg, 3),
        "camera_position_world": [round(value, 4) for value in observation.camera_position_world],
        "rotation_camera_to_world": [
            [round(value, 6) for value in row]
            for row in observation.rotation_camera_to_world
        ],
    }
    return "\n".join([
        "Visual inputs: image 1 is RGB. Image 2 is aligned depth; brighter means nearer, darker gray means farther, and black means invalid depth.",
        f"Instruction: {instruction.strip()}",
        "Camera geometry (JSON): " + json.dumps(camera, separators=(",", ":")),
        "Output exactly 5 incremental pose waypoints as a JSON list.",
        "Each waypoint must be [dx, dy, dz, dyaw_deg] relative to the previous predicted pose; the first is relative to the current vehicle pose.",
        "+dx is forward and +dy is right in the previous pose horizontal body frame. AirSim NED +dz is down. Positive dyaw_deg turns right.",
        "Translation and viewing direction are independent. Do not infer yaw from translation.",
        "Do not output any other text.",
    ])

