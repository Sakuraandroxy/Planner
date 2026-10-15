"""OpenAI-compatible grounding adapter; provider configuration is injected."""
import json
from planner.domain.target_detection import NormalizedBoundingBox, TargetDetection, TargetDetectionInput
from planner.errors import ProtocolError
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
        if data == []:
            return TargetDetection(False)
        if not isinstance(data, list) or len(data) != 2 or any(
            not isinstance(point, list) or len(point) != 2 for point in data
        ):
            raise ValueError("expected two bbox corners or []")
        return TargetDetection(True, NormalizedBoundingBox(*data[0], *data[1]))
    except (ValueError, TypeError, OverflowError) as exc:
        raise ProtocolError(f"invalid target detection bbox response: {exc}") from exc


class OpenAICompatibleTargetDetector:
    def __init__(self, transport, *, maximum_image_side=512, prompt_builder=None):
        if type(maximum_image_side) is not int or maximum_image_side <= 0:
            raise ValueError("maximum_image_side must be positive integer")
        self.transport, self.maximum_image_side = transport, maximum_image_side
        self.prompt = prompt_builder or VisualGoalTargetDetectionPrompt()

    def _image(self, image):
        result = image.copy()
        result.thumbnail((self.maximum_image_side, self.maximum_image_side))
        return result

    def detect(self, request: TargetDetectionInput, *, timeout_s: float) -> TargetDetection:
        images = ((self._image(request.current.rgb), "JPEG"),)
        if request.reference_rgb is not None:
            images += ((self._image(request.reference_rgb), "JPEG"),)
        raw = self.transport.request(TARGET_DETECTION_SYSTEM_PROMPT, self.prompt.build(request),
            images, timeout_s=timeout_s, operation="target detection")
        return parse_target_detection(raw)
