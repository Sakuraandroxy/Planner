"""Filesystem TaskTrace adapter; never writes model payloads or secrets."""
from dataclasses import asdict, is_dataclass
from datetime import datetime
import json
from pathlib import Path
import uuid

import numpy as np
from PIL import ImageDraw
from planner.domain.visual_goal import GoalObservation, GoalGeometry
from planner.domain.observation import Observation


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

    def observation(self, observation: Observation, goal: GoalObservation):
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
            if goal.surface_point is not None:
                x, y = goal.surface_point[0]*w, goal.surface_point[1]*h
                drawing.ellipse((x-4, y-4, x+4, y+4), outline="lime", width=2)
                drawing.text((x+5, y), "bbox-center", fill="yellow")
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
                   depth_mapping=observation.depth_mapping, depth_min_m=observation.depth_min_m,
                   depth_max_m=observation.depth_max_m,
                   depth_kind=observation.depth_kind, pose=observation.vehicle_pose, goal=goal)

    def target_geometry(self, observation: Observation, goal: GoalObservation, geometry: GoalGeometry) -> None:
        overlay = observation.rgb.copy()
        if goal.surface_point is not None:
            width, height = overlay.size
            x, y = goal.surface_point[0]*width, goal.surface_point[1]*height
            drawing = ImageDraw.Draw(overlay)
            color = "lime" if geometry.reliable else "red"
            drawing.ellipse((x-5, y-5, x+5, y+5), outline=color, width=2)
            label = f"bbox-center distance={geometry.distance_m:.3f}m" if geometry.distance_m is not None else "bbox-center invalid depth"
            drawing.text((x+6, y), label, fill=color)
        overlay.save(self.path/f"{self.counter:04d}_target_geometry.png")
