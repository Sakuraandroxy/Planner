"""Above-only grounding protocol over the existing multimodal transport."""
import json
import logging
from PIL import ImageDraw
from planner.domain.target_detection import NormalizedBoundingBox, TargetDetection
from planner.domain.visual_goal_above import AboveDetection, TopImageRegion, AboveDetectionMode
from planner.errors import ProtocolError
from .prompts.visual_goal_above import SYSTEM, VisualGoalAboveTargetDetectionPrompt


def unique_fields(pairs):
    result = {}
    for key,value in pairs:
        if key in result:
            raise ValueError("duplicate detection field")
        result[key] = value
    return result


def parse_above_detection(raw, *, include_top=True):
    try:
        data = json.loads(raw,object_pairs_hook=unique_fields)
        if not isinstance(data,dict) or set(data) != {"bbox","surface_points","top_regions"}:
            raise ValueError("expected bbox, surface_points, top_regions")
        points = tuple(tuple(p) for p in data["surface_points"])
        box = data["bbox"]
        if box == []:
            TargetDetection(False,surface_points=points)
            if data["top_regions"] != []:
                raise ValueError("missing target cannot supply top regions")
            return AboveDetection(None,(),())
        if not isinstance(box,list) or len(box) != 2 or any(len(p) != 2 for p in box):
            raise ValueError("bbox must contain two corners")
        bbox = NormalizedBoundingBox(*box[0],*box[1])
        TargetDetection(True,bbox,points)
        if not include_top:
            return AboveDetection(bbox,points,())
        if not isinstance(data["top_regions"],list):
            raise ValueError("top_regions must be an array")
        regions = []
        for item in data["top_regions"]:
            if not isinstance(item,dict) or set(item) != {"outline","holes","support_points"}:
                raise ValueError("top region requires outline, holes and support_points")
            region = TopImageRegion(tuple(tuple(p) for p in item["outline"]),
                                    tuple(tuple(tuple(p) for p in ring) for ring in item["holes"]),
                                    tuple(tuple(p) for p in item["support_points"]))
            regions.append(region)
        return AboveDetection(bbox,points,tuple(regions))
    except (ValueError,TypeError,KeyError,IndexError,OverflowError) as exc:
        raise ProtocolError(f"invalid above detection response: {exc}") from exc


class OpenAICompatibleTopDetector:
    def __init__(self, transport, maximum_image_side=1024):
        self.transport, self.maximum_image_side = transport, maximum_image_side
        self.prompt = VisualGoalAboveTargetDetectionPrompt()

    def detect(self, request, *, timeout_s):
        def resize(im):
            result = im.copy()
            result.thumbnail((self.maximum_image_side,self.maximum_image_side))
            return result
        current = resize(request.current.rgb)
        draw = ImageDraw.Draw(current)
        for index,point in enumerate(request.projected_target_points,1):
            u,v = point.image
            x,y = u*current.width,v*current.height
            draw.ellipse((x-3,y-3,x+3,y+3),outline="magenta",width=1)
            draw.text((x+4,y),str(index),fill="magenta")
        images = ((current,"JPEG"),)
        if request.reference_rgb is not None:
            images += ((resize(request.reference_rgb),"JPEG"),)
        raw = self.transport.request(SYSTEM,self.prompt.build(request),images,
            timeout_s=timeout_s,operation="above target detection")
        logging.getLogger(__name__).info("[AboveDetection] raw response:\n%s",raw)
        return parse_above_detection(raw,include_top=request.mode is AboveDetectionMode.TARGET_AND_TOP)
