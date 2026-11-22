"""Freeze image/depth inputs before handing them to a model worker."""
from dataclasses import replace


def freeze_observation(observation):
    depth = observation.depth_m.copy()
    depth.setflags(write=False)
    depth_image = observation.depth_image
    if depth_image is not None:
        depth_image = depth_image.copy()
    return replace(observation, rgb=observation.rgb.copy(),
                   depth_image=depth_image, depth_m=depth)


def freeze_reference(reference):
    if reference is None:
        return None
    return replace(reference, crop=reference.crop.copy() if reference.crop is not None else None)