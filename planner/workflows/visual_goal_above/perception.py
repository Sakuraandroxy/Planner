"""Model-only detection and pure geometry; collected only by the control owner."""
from dataclasses import dataclass, replace
import logging
import time
from planner.domain.observation import Observation
from planner.domain.visual_goal_above import (
    AboveDetection, AboveDetectionInput, AboveDetectionMode, AboveProjectedPoint, AboveAssociationResult, TopWorldRegion)
from planner.capabilities.targeting.above_target_association import AboveTargetAssociation, reference_points, project_points
from planner.ports.top_detector import TopDetector
from planner.services.observation_snapshot import freeze_observation

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class AbovePerceptionResult:
    frame: Observation
    detection: AboveDetection
    association: AboveAssociationResult
    regions: tuple[TopWorldRegion, ...]
    mode: AboveDetectionMode = AboveDetectionMode.TARGET_AND_TOP
    geometry_elapsed_s: float = 0.0


class AbovePerception:
    def __init__(self, detector: TopDetector, geometry, association=None):
        self.detector,self.geometry = detector,geometry
        self.association = association or AboveTargetAssociation()

    def operation(self, frame, parameters, session, timeout, *, mode=None):
        frozen = freeze_observation(frame)
        reference = session.reference_rgb.copy() if session.reference_rgb is not None else None
        points = reference_points(session.binding,session.known_regions,session.surfaces)
        binding,previous = session.binding,session.known_regions
        previous_surfaces = session.surfaces
        mode = mode or (AboveDetectionMode.TARGET_AND_TOP if session.top_detection_enabled
                        else AboveDetectionMode.TARGET_ONLY)
        request = AboveDetectionInput(frozen,parameters.instruction,parameters.target_description,reference,
            tuple(AboveProjectedPoint(point,uv) for point in points for uv in project_points(frozen,(point,))),points,mode)
        def run():
            try:
                detection = self.detector.detect(request,timeout_s=timeout)
                if mode is AboveDetectionMode.TARGET_ONLY:
                    detection = replace(detection,top_regions=())
                started = time.perf_counter()
                surfaces = self.geometry.surface_points(frozen,detection)
                regions = self.geometry.estimate(frozen,detection) if detection.top_regions else ()
                association = self.association.select(frozen,detection,surfaces,tuple(regions),binding,
                    reference_regions=previous,reference_surfaces=previous_surfaces)
                return AbovePerceptionResult(frozen,detection,association,association.regions,mode,
                                             time.perf_counter()-started)
            except Exception as exc:
                logger.warning("[AbovePerception] observation update unavailable (%s); retaining stage geometry",
                               type(exc).__name__)
                return None
        return run

    def accept(self, result, session, trace, *, frame=None):
        if result is None and frame is None:
            return False
        frame = result.frame if result is not None else frame
        detection = result.detection if result is not None else None
        association = (result.association if result is not None else
                       AboveAssociationResult(None,(),(),False,"detection_unavailable"))
        session.last_detection_capture_id = frame.capture_id
        session.observe_box(frame,association.detection)
        if detection is not None:
            trace.observation(frame,detection)
        trace.event("target_association",target_visible=association.target_visible,reason=association.reason,
                    accepted_regions=len(association.regions),
                    capture_id=frame.capture_id,pose=frame.vehicle_pose,
                    bound_capture_id=session.binding.capture_id if session.binding else None,
                    detection_mode=result.mode if result is not None else None)
        logger.info("[AbovePerception] detection_mode=%s target_visible=%s reported_top=%s association=%s",
                    result.mode.value if result else "unavailable",session.target_visible,
                    session.reported_top_visible,association.reason)
        detection,surfaces = association.detection,association.surfaces
        current = result.regions if result is not None else ()
        first_top = not session.known_regions and bool(current)
        changed = session.accept(frame,detection,surfaces,current)
        trace.event("above_perception",phase=session.phase,first_top=first_top,geometry_changed=changed,
                    top_regions=session.known_regions,surface_points_world=surfaces,destination=session.destination,
                    geometry_elapsed_s=result.geometry_elapsed_s if result is not None else None)
        if result is not None:
            logger.info("[AboveGeometry] local processing=%.3fs regions=%s outline_vertices=%s",
                result.geometry_elapsed_s,len(current),sum(len(r.outline_world) for r in current))
        logging.getLogger(__name__).info("[AbovePerception] phase=%s visible=%s top_regions=%s first_top=%s destination=%s",
            session.phase.value,detection is not None and detection.bbox is not None,len(session.known_regions),first_top,session.destination)
        return changed
