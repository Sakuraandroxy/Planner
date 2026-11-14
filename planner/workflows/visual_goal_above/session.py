"""State owned by one above stage; queue resets never reset its budget."""
from dataclasses import dataclass
import math

from planner.domain.execution import ExecutionBudget
from planner.domain.execution import ExecutionRecord
from planner.domain.pose import WorldPose
from planner.domain.observation import Observation
from planner.domain.target_detection import NormalizedBoundingBox
from planner.domain.visual_goal_above import (
    AbovePhase, AbovePlanningContext, AboveTargetBinding, TopWorldRegion, AboveTopObservation)
from planner.capabilities.viewing.bbox_top_view import bbox_top_view
from planner.capabilities.targeting.top_geometry import choose_destination, top_height
from planner.capabilities.completion.visual_goal_above import above_measurements
from planner.capabilities.targeting.above_target_association import project_points


@dataclass
class AboveSession:
    initial: WorldPose
    budget: ExecutionBudget
    height: float
    height_is_minimum: bool = False
    height_tolerance_m: float = 0.5
    phase: AbovePhase = AbovePhase.FINDING_TARGET
    reference_rgb: object = None
    surfaces: tuple = ()
    regions: tuple[TopWorldRegion, ...] = ()
    previous_regions: tuple[TopWorldRegion, ...] = ()
    destination: WorldPose | None = None
    planning_calls: int = 0
    replans: int = 0
    last_view: float = -math.inf
    last_detection: float = -math.inf
    last_check: float = -math.inf
    last_executed_before_replan: ExecutionRecord | None = None
    binding: AboveTargetBinding | None = None
    target_visible: bool = False
    last_detection_capture_id: str | None = None
    last_status: float = -math.inf
    bbox: NormalizedBoundingBox | None = None
    bbox_frame: Observation | None = None
    top_detection_enabled: bool = False
    reported_top_visible: bool = False
    top_observation: AboveTopObservation | None = None
    top_view_needed: bool = False

    @property
    def known_regions(self):
        return (*self.regions,*self.previous_regions)

    def context(self, parameters):
        return AbovePlanningContext(self.phase,parameters.target_description,self.surfaces,
                                    self.known_regions,self.destination,self.height,self.last_executed_before_replan,
                                    self.height_is_minimum,self.target_visible,self.top_detection_enabled,
                                    self.reported_top_visible,self.top_observation,
                                    self.binding.surface_points if self.binding else ())

    def observe_box(self, frame, detection):
        self.target_visible = detection is not None and detection.bbox is not None
        self.reported_top_visible = bool(detection and detection.top_regions)
        if not self.target_visible:
            return
        self.bbox,self.bbox_frame = detection.bbox,frame
        cue = bbox_top_view(frame,detection.bbox)
        self.top_observation = AboveTopObservation(frame.capture_id,frame.vehicle_pose,
            detection.bbox,cue.edge_elevations_deg,cue.clipped_top)
        previous = self.top_detection_enabled
        z = (top_height(self.known_regions,self.destination.x,self.destination.y)
             if self.destination is not None else None)
        self.top_detection_enabled = frame.vehicle_pose.z < z if z is not None else cue.top_observation_possible
        self.top_view_needed |= self.top_detection_enabled and not previous and not self.regions

    def accept(self, frame, detection, surfaces, regions):
        if detection is not None and detection.bbox is not None and self.reference_rgb is None:
            b = detection.bbox
            w,h = frame.rgb.size
            self.reference_rgb = frame.rgb.crop((int(b.xmin*w),int(b.ymin*h),
                max(int(b.xmin*w)+1,int(b.xmax*w)),max(int(b.ymin*h)+1,int(b.ymax*h))))
            if self.binding is None:
                self.binding = AboveTargetBinding(frame.capture_id,surfaces)
        if surfaces:
            self.surfaces = surfaces
        if surfaces and (self.binding is None or not self.binding.surface_points):
            self.binding = AboveTargetBinding(self.binding.capture_id if self.binding else frame.capture_id,surfaces)
        previous = self.destination
        old_z = top_height(self.known_regions,previous.x,previous.y) if previous is not None else None
        if regions:
            reference = (self.previous_regions if previous is not None and
                top_height(self.regions,previous.x,previous.y) is None else self.regions)
            self.previous_regions,self.regions = reference,regions
        if regions:
            self.top_view_needed = False
        z = top_height(self.regions,previous.x,previous.y) if previous is not None else None
        corrected = bool(regions and previous is not None and z is None and old_z is not None and
                         project_points(frame,((previous.x,previous.y,old_z),)))
        if corrected:
            self.previous_regions = ()
            self.destination = choose_destination(self.regions,frame.vehicle_pose,self.height,minimum=self.height_is_minimum)
        elif previous is None:
            self.destination = choose_destination(self.regions,frame.vehicle_pose,self.height,minimum=self.height_is_minimum)
        elif z is None:
            self.destination = previous  # Cropping or missing roof evidence is not a new destination.
        else:
            target_z = z-self.height
            if self.height_is_minimum:
                target_z = min(previous.z,target_z)
            self.destination = WorldPose(previous.x,previous.y,target_z,previous.yaw_deg)
        self.phase = (AbovePhase.APPROACHING_ABOVE if self.known_regions else
                      AbovePhase.REVEALING_TOP if self.reference_rgb is not None else AbovePhase.FINDING_TARGET)
        if previous is None or self.destination is None:
            return previous != self.destination
        # New observation geometry normally goes into the next planning call.
        # Replace the old queue only if its destination no longer has a roof.
        return corrected

    def should_verify(self, pose, settings, now):
        inside, _, height_met = above_measurements(pose,self.known_regions,self.height,
                                                   self.height_tolerance_m,self.height_is_minimum)
        return inside and height_met and now-self.last_check >= settings.completion_interval_s
