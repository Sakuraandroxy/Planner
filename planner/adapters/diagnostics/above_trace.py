"""Above-task diagnostic images and events, using the existing trace writer."""
import numpy as np
from datetime import datetime
from pathlib import Path
import logging
import threading
import uuid
from PIL import ImageDraw
from .visual_goal_trace import VisualGoalTrace


class AboveTrace(VisualGoalTrace):
    def __init__(self, output_root):
        self.path = Path(output_root)/f"{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}"
        self.counter = 0
        self._event_lock = threading.Lock()
        self._write("create output directory",self.path.mkdir,parents=True,exist_ok=False)

    @staticmethod
    def _write(operation, callback, /, *args, **kwargs):
        try:
            return callback(*args,**kwargs)
        except Exception as exc:
            logging.getLogger(__name__).warning("[AboveDiagnostics] %s failed (%s)",operation,type(exc).__name__)

    def event(self, event, **data):
        self._write("write event",super().event,event,**data)

    def planning_observation(self, request_id, observation):
        self._write("write planning images",super().planning_observation,request_id,observation)

    def control_samples(self, samples, *, dropped=0):
        self._write("write control samples",super().control_samples,samples,dropped=dropped)

    def observation(self, observation, detection):
        self._write("write observation",self._observation,observation,detection)

    def _observation(self, observation, detection):
        self.counter += 1
        stem = f"observe_{self.counter:04d}"
        observation.rgb.save(self.path/f"{stem}_rgb.jpg",quality=90)
        observation.depth_image.save(self.path/f"{stem}_depth.png")
        np.savez_compressed(self.path/f"{stem}_depth_meters.npz",depth_meters=observation.depth_meters)
        overlay = observation.rgb.copy()
        draw = ImageDraw.Draw(overlay)
        w,h = overlay.size
        if detection.bbox is not None:
            b = detection.bbox
            draw.rectangle((b.xmin*w,b.ymin*h,b.xmax*w,b.ymax*h),outline="red",width=3)
        for p in detection.surface_points:
            x,y = p[0]*w,p[1]*h
            draw.ellipse((x-4,y-4,x+4,y+4),fill="yellow")
        for region in detection.top_regions:
            for ring,color in ((region.outline,"lime"),*((hole,"orange") for hole in region.holes)):
                points = [(x*w,y*h) for x,y in ring]
                draw.line([*points,points[0]],fill=color,width=3)
            for p in region.support_points:
                x,y = p[0]*w,p[1]*h
                draw.ellipse((x-6,y-6,x+6,y+6),fill="cyan")
        overlay.save(self.path/f"{stem}_top.jpg",quality=90)
        self.event("above_observation",capture_id=observation.capture_id,detection=detection,
                   pose=observation.vehicle_pose,camera_position=observation.camera_position_world,
                   camera_rotation=observation.rotation_camera_to_world,
                   rgb_intrinsics=observation.intrinsics,depth_intrinsics=observation.depth_intrinsics,
                   depth_kind=observation.depth_kind,timestamp_ns=observation.timestamp_ns,
                   depth_timestamp_ns=observation.depth_timestamp_ns)

    def completion_observation(self, request, verdict):
        self._write("write completion evidence",self._completion_observation,request,verdict)

    def _completion_observation(self, request, verdict):
        from planner.adapters.task_completion.above_evidence import marked_above_image
        from planner.adapters.model_visual_inputs import aligned_depth_image
        stem = "completion_"+request.observation.capture_id
        request.observation.rgb.save(self.path/f"{stem}_rgb.jpg",quality=90)
        marked_above_image(request).save(self.path/f"{stem}_marked.jpg",quality=90)
        aligned_depth_image(request.observation).save(self.path/f"{stem}_depth.png")
        np.savez_compressed(self.path/f"{stem}_depth_meters.npz",depth_meters=request.observation.depth_meters)
        self.event("completion_images",capture_id=request.observation.capture_id,verdict=verdict,
                   stem=stem,body_projection_image=request.body_projection_image,
                   top_regions=request.top_regions,rgb_intrinsics=request.observation.intrinsics,
                   depth_intrinsics=request.observation.depth_intrinsics,
                   camera_position=request.observation.camera_position_world,
                   camera_rotation=request.observation.rotation_camera_to_world)
