"""Per-point native-depth discontinuities; no global bbox histogram or new points."""
from dataclasses import replace
import math
import numpy as np

from planner.domain.depth_foreground import DepthForegroundResult, DepthForegroundSettings, DepthPointLayer


class DepthForegroundFilter:
    def __init__(self, settings: DepthForegroundSettings):
        self.settings = settings

    def filter(self, observation, bbox, depth, minimum_m, samples, pixels):
        if not self.settings.enabled:
            return samples, DepthForegroundResult(False, "disabled")
        kept, layers = [], []
        rejected = 0
        for sample, pixel in zip(samples,pixels):
            layer = self._layer(observation,bbox,depth,minimum_m,pixel) if sample.valid else None
            if layer is not None:
                near_min, near_max, far_min, cutoff = layer
                layers.append(DepthPointLayer(sample.image_point, near_min, near_max, far_min, cutoff))
                k = observation.intrinsics
                x,y = int(pixel[0]),int(pixel[1])
                forward = sample.depth_m/math.sqrt(1+((x-k.cx)/k.fx)**2+((y-k.cy)/k.fy)**2)
                if forward > cutoff:
                    sample = replace(sample,valid=False,reason=sample.reason+
                        f"; filtered_background; local_forward_m={forward:.6g}; cutoff_m={cutoff:.6g}")
                    rejected += 1
            kept.append(sample)
        return tuple(kept),DepthForegroundResult(bool(layers),
            "local_depth_discontinuity_not_semantic_segmentation" if layers else "no_clear_local_depth_separation",
            rejected,tuple(layers))

    def _layer(self,obs,bbox,depth,minimum_m,pixel):
        x,y = int(pixel[0]),int(pixel[1])
        rgb,k = obs.intrinsics,obs.intrinsics
        radius = self.settings.pixel_radius
        x0,x1 = max(0,x-radius),min(k.width,x+radius+1)
        y0,y1 = max(0,y-radius),min(k.height,y+radius+1)
        yy,xx = np.ogrid[y0:y1,x0:x1]
        raw = np.asarray(depth[y0:y1,x0:x1],dtype=float)
        u = ((xx-k.cx)/k.fx*rgb.fx+rgb.cx)/rgb.width
        v = ((yy-k.cy)/k.fy*rgb.fy+rgb.cy)/rgb.height
        valid = np.isfinite(raw)&(raw>=minimum_m)&(raw>0)&(u>=bbox[0])&(u<=bbox[2])&(v>=bbox[1])&(v<=bbox[3])
        forward = raw/np.sqrt(1+((xx-k.cx)/k.fx)**2+((yy-k.cy)/k.fy)**2)
        values = np.sort(forward[valid])
        if len(values)<2:
            return None
        differences = np.diff(values)
        index = int(np.argmax(differences))
        spatial = np.concatenate((np.abs(np.diff(forward,axis=1))[valid[:,1:]&valid[:,:-1]],
                                  np.abs(np.diff(forward,axis=0))[valid[1:,:]&valid[:-1,:]]))
        if not spatial.size:
            return None
        numerical = np.finfo(float).eps*max(1.,float(values[-1]))*64
        if differences[index] <= max(numerical,self.settings.depth_gap_ratio*float(np.median(spatial))):
            return None
        cutoff = float((values[index]+values[index+1])/2)
        near = valid&(forward<=cutoff)
        far = valid&(forward>cutoff)
        adjacent = lambda mask: bool(np.any(mask[:,1:]&mask[:,:-1]) or np.any(mask[1:,:]&mask[:-1,:]))
        if not adjacent(near) or not adjacent(far):
            return None
        return float(values[0]),float(values[index]),float(values[index+1]),cutoff