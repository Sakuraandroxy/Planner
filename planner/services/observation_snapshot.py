"""Freeze image/depth inputs before handing them to a model worker."""
from dataclasses import replace


def freeze_observation(observation):
    depth = observation.depth_meters.copy()
    depth.setflags(write=False)
    return replace(observation, rgb=observation.rgb.copy(),
                   depth_image=observation.depth_image.copy(), depth_meters=depth)


def freeze_reference(reference):
    if reference is None:
        return None
    return replace(reference, crop=reference.crop.copy())
