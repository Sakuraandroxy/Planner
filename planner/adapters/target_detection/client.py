"""OpenAI-compatible grounding adapter; provider configuration is injected."""
import json
from planner.domain.target_detection import NormalizedBoundingBox, TargetDetection, TargetDetectionInput
from planner.errors import ProtocolError
from ..image_encoding import resize_longest_side
from .prompts import TARGET_DETECTION_SYSTEM_PROMPT, VisualGoalTargetDetectionPrompt


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate target detection field")
        result[key] = value
    return result


def parse_target_detection(raw: str) -> TargetDetection:
    try:
        data = json.loads(raw, object_pairs_hook=_unique_object)
        if not isinstance(data, dict) or set(data) != {"bbox", "surface_points"}:
            raise ValueError("expected exactly bbox and surface_points fields")
        box, points = data["bbox"], data["surface_points"]
        if not isinstance(points, list) or any(not isinstance(p, list) or len(p) != 2 for p in points):
            raise ValueError("surface_points must be an array of coordinate pairs")
        samples = tuple(tuple(p) for p in points)
        if box == []:
            return TargetDetection(False, surface_points=samples)
        if not isinstance(box, list) or len(box) != 2 or any(
            not isinstance(point, list) or len(point) != 2 for point in box
        ):
            raise ValueError("bbox requires two corners or []")
        return TargetDetection(True, NormalizedBoundingBox(*box[0], *box[1]), samples)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ProtocolError(f"invalid target detection surface response: {exc}") from exc


class OpenAICompatibleTargetDetector:
    def __init__(self, transport, *, maximum_image_side=1024, prompt_builder=None):
        if type(maximum_image_side) is not int or maximum_image_side <= 0:
            raise ValueError("maximum_image_side must be positive integer")
        self.transport, self.maximum_image_side = transport, maximum_image_side
        self.prompt = prompt_builder or VisualGoalTargetDetectionPrompt()

    def _image(self, image):
        # CARLA observations carry RGB as a numpy array; resize in place of
        # the reference Pillow thumbnail, keeping the longest side bounded.
        return resize_longest_side(image, self.maximum_image_side)

    def detect(self, request: TargetDetectionInput, *, timeout_s: float) -> TargetDetection:
        images = ((self._image(request.current.rgb), "JPEG"),)
        if request.reference_rgb is not None:
            images += ((self._image(request.reference_rgb), "JPEG"),)
        raw = self.transport.request(TARGET_DETECTION_SYSTEM_PROMPT, self.prompt.build(request),
            images, timeout_s=timeout_s, operation="target detection")
        return parse_target_detection(raw)