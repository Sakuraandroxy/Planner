"""Filesystem TaskTrace adapter; never writes model payloads or secrets."""
from dataclasses import asdict, is_dataclass
from datetime import datetime
import json
from pathlib import Path
import uuid

import numpy as np
from PIL import ImageDraw


class VisualGoalTrace:
    def __init__(self, output_root):
        self.path = Path(output_root)/f"{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}"
        self.path.mkdir(parents=True, exist_ok=False)
        self.counter = 0

    def event(self, event, **data):
        def serialize(value):
            if is_dataclass(value):
                return asdict(value)
            if hasattr(value, "value"):
                return value.value
            raise TypeError(type(value).__name__)
        with (self.path/"events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"event": event, "logged_wall_time": datetime.now().isoformat(), **data},
                                    default=serialize, ensure_ascii=False,
                                    allow_nan=False)+"\n")

    def observation(self, observation, goal):
        self.counter += 1
        stem = f"{self.counter:04d}"
        observation.rgb.save(self.path/f"{stem}_rgb.png")
        observation.depth_image.save(self.path/f"{stem}_depth.png")
        np.save(self.path/f"{stem}_depth_meters.npy", observation.depth_meters, allow_pickle=False)
        overlay = observation.rgb.copy()
        if goal.bbox:
            w, h = overlay.size
            a, b, c, d = goal.bbox
            drawing = ImageDraw.Draw(overlay)
            drawing.rectangle((a*w, b*h, c*w, d*h), outline="red", width=2)
            if goal.surface_point:
                x, y = goal.surface_point[0]*w, goal.surface_point[1]*h
                drawing.ellipse((x-3, y-3, x+3, y+3), outline="lime", width=2)
        overlay.save(self.path/f"{stem}_target.png")
        self.event("observation", capture_id=observation.capture_id, timestamp_ns=observation.timestamp_ns,
                   depth_timestamp_ns=observation.depth_timestamp_ns,
                   captured_monotonic=observation.captured_monotonic,
                   capture_started_monotonic=observation.capture_started_monotonic,
                   capture_finished_monotonic=observation.capture_finished_monotonic,
                   stopped=observation.stopped,
                   camera_position=observation.camera_position_world,
                   camera_rotation=observation.rotation_camera_to_world,
                   rgb_intrinsics=observation.intrinsics, depth_intrinsics=observation.depth_intrinsics,
                   depth_kind=observation.depth_kind, pose=observation.vehicle_pose, goal=goal)
