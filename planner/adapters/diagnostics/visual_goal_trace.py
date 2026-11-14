"""Filesystem TaskTrace adapter; never writes model payloads or secrets."""
from dataclasses import asdict, is_dataclass
from datetime import datetime
import csv
import json
import math
from pathlib import Path
import threading
import uuid

import cv2
import numpy as np

from planner.domain.observation import Observation
from planner.domain.visual_goal import GoalGeometry, GoalObservation


def _write_image(path: Path, image, *, quality: int | None = None) -> None:
    """Write an RGB visualisation or a single-channel depth map to disk."""
    array = np.ascontiguousarray(np.asarray(image, dtype=np.uint8))
    if array.ndim == 3 and array.shape[2] == 3:
        array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    params = [] if quality is None else [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
    if not cv2.imwrite(str(path), array, params):
        raise OSError(f"could not write image {path}")


def _text(drawing, text: str, position, color, *, stroke: bool = False) -> None:
    if stroke:
        cv2.putText(drawing, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(drawing, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                color, 1, cv2.LINE_AA)


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
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, np.generic):
                return value.item()
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
        _write_image(self.path/f"{stem}_rgb.jpg", observation.rgb, quality=90)
        _write_image(self.path/f"{stem}_depth.png", aligned_depth_image(observation))
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
        _write_image(self.path/f"{stem}_rgb.png", observation.rgb)
        _write_image(self.path/f"{stem}_depth.png", observation.depth_image)
        np.save(self.path/f"{stem}_depth_meters.npy", observation.depth_m, allow_pickle=False)
        overlay = cv2.cvtColor(np.ascontiguousarray(observation.rgb.copy()), cv2.COLOR_RGB2BGR)
        if goal.bbox:
            h, w = overlay.shape[:2]
            a, b, c, d = goal.bbox
            cv2.rectangle(overlay, (int(a*w), int(b*h)), (int(c*w), int(d*h)), (0, 0, 255), 2)
            points = goal.surface_points if goal.surface_points is not None else (
                (goal.surface_point,) if goal.surface_point is not None else ())
            for index, point in enumerate(points, 1):
                x, y = int(point[0]*w), int(point[1]*h)
                cv2.circle(overlay, (x, y), 4, (0, 255, 0), 2)
                _text(overlay, f"surface-{index}", (x+5, y), (0, 255, 255))
        _write_image(self.path/f"{stem}_target.png", overlay)
        self.event("observation", capture_id=observation.capture_id, timestamp_ns=observation.timestamp_ns,
                   depth_timestamp_ns=observation.depth_timestamp_ns,
                   captured_monotonic=observation.captured_monotonic,
                   stopped=observation.stopped,
                   camera_position=observation.camera_position_world,
                   camera_rotation=observation.rotation_camera_to_world,
                   rgb_intrinsics=observation.intrinsics, depth_intrinsics=observation.depth_intrinsics,
                   depth_mapping=observation.depth_mapping, depth_min_m=observation.depth_min_m,
                   depth_max_m=observation.depth_max_m,
                   depth_kind=observation.depth_kind, pose=observation.pose, goal=goal)

    def target_geometry(self, observation: Observation, goal: GoalObservation, geometry: GoalGeometry) -> None:
        overlay = cv2.cvtColor(np.ascontiguousarray(observation.rgb.copy()), cv2.COLOR_RGB2BGR)
        height, width = overlay.shape[:2]
        for index, sample in enumerate(geometry.surface_samples, 1):
            x, y = int(sample.image_point[0]*width), int(sample.image_point[1]*height)
            color = (0, 255, 0) if sample.valid else (0, 0, 255)
            cv2.circle(overlay, (x, y), 5, color, 2)
            selected = sample.valid and sample.world_point == geometry.world_point
            label = (f"surface-{index} distance={sample.distance_m:.3f}m" if sample.valid else
                     f"surface-{index} "+("background filtered" if "filtered_background" in sample.reason else "invalid depth"))
            if selected:
                label += " selected"
            _text(overlay, label, (x+6, y), color)
        if geometry.depth_filter is not None:
            filtering = geometry.depth_filter
            _text(overlay, f"Depth filter: {filtering.reason}", (8, 16), (0, 255, 255), stroke=True)
            if filtering.applied:
                _text(overlay, f"Local layers: {filtering.point_layers}; rejected: {filtering.rejected_points}",
                      (8, 32), (0, 255, 255), stroke=True)
        _write_image(self.path/f"{self.counter:04d}_target_geometry.png", overlay)
        self.event("surface_depth_samples", capture_id=observation.capture_id,
            position_source=geometry.position_source, samples=geometry.surface_samples,
            sample_distance_mode="3d_sample_distance; task_distance_uses_horizontal_bound_surfaces",
            selected_world_point=geometry.world_point, selected_distance_m=geometry.distance_m,
            depth_filter=geometry.depth_filter)