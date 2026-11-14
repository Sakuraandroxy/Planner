import numpy as np
import pytest

from planner.adapters.target_detection import (
    ManualTargetDetector,
    OpenAICompatibleTargetDetector,
    parse_target_detection,
)
from planner.adapters.target_detection.prompts import (
    TARGET_DETECTION_SYSTEM_PROMPT,
    VisualGoalTargetDetectionPrompt,
)
from planner.domain.observation import CameraIntrinsics, Observation
from planner.domain.pose import WorldPose
from planner.domain.target_detection import (
    NormalizedBoundingBox,
    TargetDetection,
    TargetDetectionInput,
)
from planner.errors import ProtocolError


def _observation() -> Observation:
    intrinsics = CameraIntrinsics.from_horizontal_fov(640, 480, 90.0)
    return Observation(
        frame=1,
        timestamp_s=0.0,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.full((480, 640), 10.0, dtype=np.float32),
        intrinsics=intrinsics,
        camera_matrix=np.eye(4),
    )


def test_parse_valid_detection():
    detection = parse_target_detection(
        '{"bbox":[[0.1,0.2],[0.5,0.6]],"surface_points":[[0.3,0.4],[0.4,0.5]]}'
    )
    assert detection.found
    assert detection.bbox == NormalizedBoundingBox(0.1, 0.2, 0.5, 0.6)
    assert detection.surface_points == ((0.3, 0.4), (0.4, 0.5))


def test_parse_missing_detection():
    detection = parse_target_detection('{"bbox":[],"surface_points":[]}')
    assert not detection.found
    assert detection.bbox is None
    assert detection.surface_points == ()


def test_parse_rejects_duplicate_fields():
    with pytest.raises(ProtocolError):
        parse_target_detection(
            '{"bbox":[[0.1,0.1],[0.2,0.2]],"bbox":[[0.1,0.1],[0.2,0.2]],'
            '"surface_points":[]}'
        )


def test_parse_rejects_extra_fields():
    with pytest.raises(ProtocolError):
        parse_target_detection(
            '{"bbox":[[0.1,0.1],[0.2,0.2]],"surface_points":[],"depth":3}'
        )


def test_parse_rejects_point_outside_bbox():
    with pytest.raises(ProtocolError):
        parse_target_detection(
            '{"bbox":[[0.1,0.1],[0.2,0.2]],"surface_points":[[0.8,0.8]]}'
        )


def test_parse_rejects_unordered_bbox():
    with pytest.raises(ProtocolError):
        parse_target_detection(
            '{"bbox":[[0.6,0.6],[0.2,0.2]],"surface_points":[]}'
        )


def test_manual_detector_returns_fixed_box():
    detector = ManualTargetDetector.from_corners(0.4, 0.4, 0.6, 0.6)
    request = TargetDetectionInput(_observation(), "go", "car")
    detection = detector.detect(request, timeout_s=1.0)
    assert detection.usable_points == ((0.5, 0.5),)


class _FakeTransport:
    def __init__(self, text: str):
        self.text = text
        self.calls = []

    def request(
        self, system, text, images, *, timeout_s, operation, **kwargs
    ):
        self.calls.append((system, text, images, timeout_s, operation))
        return self.text


def _reference_rgb() -> np.ndarray:
    return np.zeros((100, 200, 3), dtype=np.uint8)


def test_detector_adapter_sends_images_and_parses_response():
    transport = _FakeTransport(
        '{"bbox":[[0.4,0.4],[0.6,0.6]],"surface_points":[]}'
    )
    detector = OpenAICompatibleTargetDetector(transport)
    request = TargetDetectionInput(_observation(), "go to the car", "car")
    detection = detector.detect(request, timeout_s=5.0)
    assert detection.found
    system, text, images, timeout_s, operation = transport.calls[0]
    assert system == TARGET_DETECTION_SYSTEM_PROMPT
    assert operation == "target detection"
    assert timeout_s == 5.0
    assert "go to the car" in text
    assert len(images) == 1
    image, image_format = images[0]
    assert image.shape == (480, 640, 3)
    assert image_format == "JPEG"


def test_detector_adapter_adds_reference_crop_as_second_image():
    transport = _FakeTransport(
        '{"bbox":[[0.4,0.4],[0.6,0.6]],"surface_points":[]}'
    )
    detector = OpenAICompatibleTargetDetector(transport)
    request = TargetDetectionInput(
        _observation(), "go to the car", "car", _reference_rgb()
    )
    detector.detect(request, timeout_s=5.0)
    _, text, images, _, _ = transport.calls[0]
    assert len(images) == 2
    assert "Image 2: bound-target reference crop" in text
    assert "do not substitute another object" in text


def test_detector_adapter_downscales_the_longest_side():
    transport = _FakeTransport(
        '{"bbox":[[0.4,0.4],[0.6,0.6]],"surface_points":[]}'
    )
    detector = OpenAICompatibleTargetDetector(transport, maximum_image_side=320)
    request = TargetDetectionInput(_observation(), "go", "car")
    detector.detect(request, timeout_s=5.0)
    image, _ = transport.calls[0][2][0]
    assert image.shape == (240, 320, 3)


def test_detector_adapter_validates_maximum_image_side():
    with pytest.raises(ValueError):
        OpenAICompatibleTargetDetector(_FakeTransport("{}"), maximum_image_side=0)


def test_detection_prompts_match_the_reference_protocol():
    prompt = VisualGoalTargetDetectionPrompt().build(
        TargetDetectionInput(_observation(), "go to the car", "car")
    )
    assert prompt.startswith("Task: go to the car\nTarget: car")
    assert '{"bbox":[[x1,y1],[x2,y2]],"surface_points":[[x,y],...]}' in prompt
    assert "empty surface_points array" in prompt


def test_detection_rejects_inconsistent_state():
    with pytest.raises(ValueError):
        TargetDetection(found=True)
    with pytest.raises(ValueError):
        TargetDetection(
            found=False, bbox=NormalizedBoundingBox(0.1, 0.1, 0.2, 0.2)
        )