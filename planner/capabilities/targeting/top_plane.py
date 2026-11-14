"""Sparse roof planes and contour-ray intersections; no depth raster meshing."""
from itertools import combinations
import logging
import numpy as np

from planner.domain.above_polygon import in_ring
from planner.domain.visual_goal_above import TopWorldRegion

logger = logging.getLogger(__name__)


def fit_plane(points):
    if len(points) < 3:
        return None
    points = np.asarray(points,dtype=float)
    center = points.mean(axis=0)
    _,singular,vectors = np.linalg.svd(points-center,full_matrices=False)
    if singular[1] <= 1e-12 or abs(vectors[-1,2]) <= 1e-12:
        return None  # Collinear points or a vertical plane cannot define roof z(x,y).
    normal = vectors[-1]
    return tuple(map(float,(*normal,-normal@center)))


def project_ring(obs, ring, plane):
    k = obs.intrinsics
    rays = np.array([(1.,(u*k.width-k.cx)/k.fx,(v*k.height-k.cy)/k.fy) for u,v in ring])
    directions = rays@np.asarray(obs.rotation_camera_to_world).T
    normal = np.asarray(plane[:3])
    denominators = directions@normal
    if np.any(np.abs(denominators) <= 1e-12):
        return None
    distances = -(normal@np.asarray(obs.camera_position_world)+plane[3])/denominators
    if np.any(distances <= 0) or not np.isfinite(distances).all():
        return None
    return tuple(tuple(map(float,p)) for p in np.asarray(obs.camera_position_world)+directions*distances[:,None])


def _crosses(a,b,c,d):
    def side(p,q,r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    return side(a,b,c)*side(a,b,d) < -1e-18 and side(c,d,a)*side(c,d,b) < -1e-18


def _support_triangle(region, pixels):
    """Keep a finite observed portion if a contour reaches the plane's horizon."""
    boundaries = [(a,b) for ring in (region.outline,*region.holes) for a,b in zip(ring,(*ring[1:],ring[0]))]
    candidates = []
    for points in combinations(pixels,3):
        a,b,c = points
        area = abs((b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]))
        edges = tuple(zip(points,(*points[1:],points[0])))
        if area <= 1e-12:
            continue
        if any(_crosses(a,b,c,d) for a,b in edges for c,d in boundaries):
            continue
        mids = [((a[0]+b[0])/2,(a[1]+b[1])/2) for a,b in edges]
        if not all(in_ring(*p,region.outline) and not any(in_ring(*p,h) for h in region.holes) for p in mids):
            continue
        if any(in_ring(*p,points) for h in region.holes for p in h):
            continue
        candidates.append((area,points))
    return max(candidates,key=lambda item:item[0])[1] if candidates else None


def reconstruct_region(obs, region, pixels, supports):
    plane = fit_plane(supports)
    if plane is None:
        return None
    outline = project_ring(obs,region.outline,plane)
    holes = tuple(project_ring(obs,h,plane) for h in region.holes)
    if outline is None or any(h is None for h in holes):
        triangle = _support_triangle(region,pixels)
        outline = project_ring(obs,triangle,plane) if triangle is not None else None
        holes = ()
        logger.debug("Roof contour is not fully projectable; using its finite support portion")
    if outline is None:
        return None
    return TopWorldRegion(outline,holes,plane,obs.capture_id,tuple(tuple(map(float,p)) for p in supports))
