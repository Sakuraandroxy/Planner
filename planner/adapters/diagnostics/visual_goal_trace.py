"""Filesystem TaskTrace adapter; never writes model payloads or secrets."""
from dataclasses import asdict, is_dataclass
from datetime import datetime
import json
from pathlib import Path
import uuid
import threading
import csv
import math

import numpy as np
from PIL import ImageDraw
from planner.domain.visual_goal import GoalObservation, GoalGeometry
from planner.domain.observation import Observation


class VisualGoalTrace:
    def __init__(self, output_root):
        self.path = Path(output_root)/f"{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}"
        self.path.mkdir(parents=True, exist_ok=False)
        self.counter = 0
        self._event_lock = threading.Lock()

    def event(self, event, **data):
        def serialize(value):
            if is_dataclass(value):
                return asdict(value)
            if hasattr(value, "value"):
                return value.value
            raise TypeError(type(value).__name__)
        with self._event_lock, (self.path/"events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"event": event, "logged_wall_time": datetime.now().isoformat(), **data},
                                    default=serialize, ensure_ascii=False,
                                    allow_nan=False)+"\n")

    def planning_observation(self, request_id, observation):
        from planner.adapters.model_visual_inputs import aligned_depth_image
        stem = f"plan_{request_id:04d}"
        observation.rgb.convert("RGB").save(self.path/f"{stem}_rgb.jpg", quality=90)
        aligned_depth_image(observation).save(self.path/f"{stem}_depth.png")
        self.event("planning_images", request_id=request_id, capture_id=observation.capture_id,
                   rgb_file=f"{stem}_rgb.jpg", depth_file=f"{stem}_depth.png")

    def control_samples(self, samples, *, dropped=0):
        """Flush the owner's bounded in-memory samples after the vehicle stops."""
        rows = []
        for sample in samples:
            row = asdict(sample)
            for name in ('commanded_velocity', 'measured_velocity', 'commanded_acceleration'):
                vector = row.pop(name)
                row.update({name+'_'+axis: value for axis, value in zip('xyz', vector)})
                row[name+'_norm'] = math.hypot(*vector)
            rows.append(row)
        path = self.path/'control_samples.csv'
        if rows:
            existing = path.exists()
            with path.open('a', encoding='utf-8', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                if not existing:
                    writer.writeheader()
                writer.writerows(rows)
        self.event('control_telemetry', file=path.name, samples=len(rows), dropped=dropped)

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
            points = goal.surface_points if goal.surface_points is not None else (
                (goal.surface_point,) if goal.surface_point is not None else ())
            for index, point in enumerate(points, 1):
                x, y = point[0]*w, point[1]*h
                drawing.ellipse((x-4, y-4, x+4, y+4), outline="lime", width=2)
                drawing.text((x+5, y), f"surface-{index}", fill="yellow")
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
        width, height = overlay.size
        drawing = ImageDraw.Draw(overlay)
        for index, sample in enumerate(geometry.surface_samples, 1):
            x, y = sample.image_point[0]*width, sample.image_point[1]*height
            color = "lime" if sample.valid else "red"
            drawing.ellipse((x-5, y-5, x+5, y+5), outline=color, width=2)
            selected = sample.valid and sample.world_point == geometry.world_point
            label = (f"surface-{index} distance={sample.distance_m:.3f}m" if sample.valid else
                     f"surface-{index} "+("background filtered" if "filtered_background" in sample.reason else "invalid depth"))
            if selected:
                label += " selected"
            drawing.text((x+6, y), label, fill=color)
        if geometry.depth_filter is not None:
            filtering = geometry.depth_filter
            drawing.text((8, 8), f"Depth filter: {filtering.reason}", fill="yellow", stroke_width=1, stroke_fill="black")
            if filtering.applied:
                drawing.text((8, 24), f"Local layers: {filtering.point_layers}; rejected: {filtering.rejected_points}",
                    fill="yellow", stroke_width=1, stroke_fill="black")
        overlay.save(self.path/f"{self.counter:04d}_target_geometry.png")
        self.event("surface_depth_samples", capture_id=observation.capture_id,
            position_source=geometry.position_source, samples=geometry.surface_samples,
            sample_distance_mode="3d_sample_distance; task_distance_uses_horizontal_bound_surfaces",
            selected_world_point=geometry.world_point, selected_distance_m=geometry.distance_m,
            depth_filter=geometry.depth_filter)
