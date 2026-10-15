"""BBox-only model reply fixtures; no runtime target point selection."""
import json


def detection_json(bbox=((.25, .25), (.75, .75))):
    return json.dumps([list(corner) for corner in bbox])
