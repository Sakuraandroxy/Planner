"""Keep the detector's reference-image identity without freezing a roof mesh."""
import numpy as np

from planner.domain.visual_goal_above import AboveAssociationResult


def project_points(observation, points):
    """Normalized RGB coordinates, using the image-time camera extrinsics."""
    camera = np.asarray(observation.camera_position_world)
    rotation = np.asarray(observation.rotation_camera_to_world)
    k = observation.intrinsics
    result = []
    for point in points:
        p = rotation.T @ (np.asarray(point)-camera)
        if p[0] <= 0:
            continue
        u, v = (k.fx*p[1]/p[0]+k.cx)/k.width, (k.fy*p[2]/p[0]+k.cy)/k.height
        if 0 <= u <= 1 and 0 <= v <= 1:
            result.append((float(u),float(v)))
    return tuple(result)


def binding_points(binding):
    return binding.surface_points if binding is not None else ()


def reference_points(binding, regions=(), surfaces=()):
    """Initial anchors, last accepted samples and compact known-roof samples."""
    points = list(binding_points(binding))
    points.extend(surfaces)
    for region in regions:
        points.extend(region.support_points or region.outline_world[::max(1,len(region.outline_world)//8)])
    return tuple(dict.fromkeys(points))


class AboveTargetAssociation:
    def select(self, frame, detection, surfaces, regions, binding, *, reference_regions=(),
               reference_surfaces=()):
        if detection is None or detection.bbox is None:
            return AboveAssociationResult(None,(),(),False,"target_not_observed")
        if binding is not None:
            points = reference_points(binding,reference_regions,reference_surfaces)
            projected = project_points(frame,points)
            box = detection.bbox
            if points and not projected:
                return AboveAssociationResult(None,(),(),False,"bound_target_out_of_view")
            if projected and not any(box.xmin <= u <= box.xmax and box.ymin <= v <= box.ymax
                                     for u,v in projected):
                return AboveAssociationResult(None,(),(),False,"bbox_conflicts_with_bound_target")
        # Projection supports the detector's reference-image association; it
        # does not prove identity or restrict a new roof to an old roof's size.
        reason = "initial_target" if binding is None else "bound_reference_target"
        return AboveAssociationResult(detection,surfaces,regions,True,reason)
